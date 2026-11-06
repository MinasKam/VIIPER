package usb

import (
	"context"
	"encoding/binary"
	"fmt"
	"net"
	"sync"
	"time"

	"github.com/Alia5/VIIPER/usb"
	"github.com/Alia5/VIIPER/usbip"
)

const (
	// numberOfPackets value of a transfer that is not isochronous.
	isoNotIso = 0xFFFFFFFF
	// usbip_iso_packet_descriptor: offset, length, actual_length, status.
	isoDescSize = 16
	// A stream that paused longer than this restarts its clock from now.
	isoResyncAfter = 50 * time.Millisecond
)

// isoStream handles the isochronous transfers of one URB stream. Each transfer
// is consumed (OUT) or filled (IN) on arrival and completed when its packets'
// time is up, as a real bus would, so audio clocks on the host keep real time.
type isoStream struct {
	server  *Server
	conn    net.Conn
	dev     usb.Device
	ctx     context.Context
	pending map[uint32]context.CancelFunc
	pendMu  *sync.Mutex
	write   func([]byte) error

	mu    sync.Mutex
	next  map[uint32]time.Time     // per endpoint and direction: when the queue drains
	prev  map[uint32]chan struct{} // per endpoint and direction: the last transfer's completion
	frame uint32
}

func (s *isoStream) submit(seq, ep, dir, numPackets, xferLen uint32, out []byte) error {
	descs := make([]byte, int(numPackets)*isoDescSize)
	if err := usbip.ReadExactly(s.conn, descs); err != nil {
		return fmt.Errorf("read iso packet descriptors: %w", err)
	}

	buf := make([]byte, xferLen)
	if dir == usbip.DirOut {
		copy(buf, out)
	}
	packets := make([][]byte, numPackets)
	var total uint32
	for i := range packets {
		d := descs[i*isoDescSize:]
		off := binary.BigEndian.Uint32(d[0:4])
		length := binary.BigEndian.Uint32(d[4:8])
		if uint64(off)+uint64(length) > uint64(xferLen) {
			length = 0
		}
		packets[i] = buf[off : off+length]
		binary.BigEndian.PutUint32(d[8:12], length) // actual_length
		binary.BigEndian.PutUint32(d[12:16], 0)     // status
		total += length
	}

	if iso, ok := s.dev.(usb.IsoDevice); ok {
		if dir == usbip.DirOut {
			iso.HandleIsoOut(ep, packets)
		} else {
			iso.HandleIsoIn(ep, packets)
		}
	}

	period := isoPeriod(s.dev.GetDescriptor(), ep, dir)
	key := ep | dir<<8
	now := time.Now()
	s.mu.Lock()
	if s.next == nil {
		s.next = map[uint32]time.Time{}
		s.prev = map[uint32]chan struct{}{}
	}
	start := s.next[key]
	if start.Before(now.Add(-isoResyncAfter)) {
		start = now
	}
	deadline := start.Add(time.Duration(numPackets) * period)
	s.next[key] = deadline
	prev := s.prev[key]
	done := make(chan struct{})
	s.prev[key] = done
	startFrame := s.frame
	s.frame += numPackets
	s.mu.Unlock()

	urbCtx, cancel := context.WithCancel(s.ctx)
	s.pendMu.Lock()
	s.pending[seq] = cancel
	s.pendMu.Unlock()

	go func() {
		defer close(done)
		defer cancel()
		if prev != nil {
			select {
			case <-prev: // complete in submission order
			case <-urbCtx.Done():
				return
			}
		}
		t := time.NewTimer(time.Until(deadline))
		defer t.Stop()
		select {
		case <-t.C:
		case <-urbCtx.Done():
			return // unlinked or stream closed: RET_UNLINK answers it
		}

		s.pendMu.Lock()
		_, live := s.pending[seq]
		delete(s.pending, seq)
		s.pendMu.Unlock()
		if !live {
			return
		}

		var data []byte
		if dir == usbip.DirIn {
			data = buf[:total]
		}
		ret := usbip.RetSubmit{
			Basic:           usbip.HeaderBasic{Command: usbip.RetSubmitCode, Seqnum: seq},
			ActualLength:    total,
			StartFrame:      startFrame,
			NumberOfPackets: numPackets,
		}
		msg := make([]byte, 0, retSubmitHeaderSize+len(data)+len(descs))
		w := byteWriter{&msg}
		if err := ret.Write(w); err != nil {
			s.server.logger.Error("build iso RET_SUBMIT", "seq", seq, "error", err)
			return
		}
		msg = append(msg, data...)
		msg = append(msg, descs...)
		if err := s.write(msg); err != nil && !isClientDisconnect(err) {
			s.server.logger.Error("write iso RET_SUBMIT", "seq", seq, "error", err)
		}
	}()
	return nil
}

// isoPeriod is the time one packet of an isochronous endpoint stands for.
func isoPeriod(desc *usb.Descriptor, ep, dir uint32) time.Duration {
	addr := uint8(ep)
	if dir == usbip.DirIn {
		addr |= 0x80
	}
	for _, iface := range desc.Interfaces {
		for _, e := range iface.Endpoints {
			if e.BEndpointAddress != addr || e.BInterval == 0 {
				continue
			}
			if desc.Device.Speed >= 3 {
				return 125 * time.Microsecond << (e.BInterval - 1)
			}
			return time.Millisecond << (e.BInterval - 1)
		}
	}
	return time.Millisecond
}

type byteWriter struct{ b *[]byte }

func (w byteWriter) Write(p []byte) (int, error) {
	*w.b = append(*w.b, p...)
	return len(p), nil
}
