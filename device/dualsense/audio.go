package dualsense

import (
	"encoding/binary"
	"sync"

	"github.com/Alia5/VIIPER/device"
	"github.com/Alia5/VIIPER/usb"
)

// USB audio, as on a wired DualSense: a 4-channel 48 kHz speaker stream whose
// back two channels drive the haptic actuators, and a stereo microphone. Games
// that use DualSense haptics (instead of classic rumble) play them into the
// speaker stream. The interfaces, terminals and endpoints follow the physical
// controller (and https://github.com/hbashton/VIIPER, which first added them).
const (
	EndpointHapticsAudioOut = 0x01
	EndpointMicrophoneIn    = 0x82

	InterfaceAudioControl = 0x00
	InterfaceHapticsAudio = 0x01
	InterfaceMicrophone   = 0x02
	InterfaceHIDAudio     = 0x03

	AudioSampleRate     = 48000
	SpeakerChannels     = 4
	MicrophoneChannels  = 2
	audioBytesPerSample = 2

	// One millisecond per packet, plus room for one extra frame when the host
	// adjusts its rate.
	speakerMaxPacketSize    = (AudioSampleRate/1000 + 1) * SpeakerChannels * audioBytesPerSample
	microphoneMaxPacketSize = (AudioSampleRate/1000 + 1) * MicrophoneChannels * audioBytesPerSample
)

const (
	audioRequestSetCurrent    = 0x01
	audioRequestGetCurrent    = 0x81
	audioRequestGetMinimum    = 0x82
	audioRequestGetMaximum    = 0x83
	audioRequestGetResolution = 0x84

	audioControlMute   = 0x01
	audioControlVolume = 0x02

	usbRequestGetInterface = 0x0A
	usbRequestSetInterface = 0x0B
)

// HapticsFunc receives the haptics channels of the speaker stream as
// interleaved left/right signed 16-bit samples at 48 kHz.
type HapticsFunc func(samples []int16)

type audioState struct {
	mtx         sync.Mutex
	hapticsFunc HapticsFunc
	alt         map[uint16]uint8
	mute        map[uint16]uint8 // by feature unit
	volume      map[uint16]int16 // by feature unit
	sampleRate  map[uint16][]byte
	scratch     []int16
}

// NewEdgeWithAudio creates a DualSense Edge that also has the controller's USB
// audio device, so games can send it haptics.
func NewEdgeWithAudio(o *device.CreateOptions) (*DualSense, error) {
	d, err := new(o, true)
	if err != nil {
		return nil, err
	}
	d.descriptor = withAudio(d.descriptor)
	d.audio = &audioState{
		alt:        map[uint16]uint8{},
		mute:       map[uint16]uint8{},
		volume:     map[uint16]int16{},
		sampleRate: map[uint16][]byte{},
	}
	return d, nil
}

// SetHapticsCallback sets the receiver of the haptics channels. Only devices
// created with audio ever call it.
func (d *DualSense) SetHapticsCallback(f HapticsFunc) {
	if d.audio == nil {
		return
	}
	d.audio.mtx.Lock()
	d.audio.hapticsFunc = f
	d.audio.mtx.Unlock()
}

// withAudio puts the audio interfaces in front of the HID interface, which
// becomes interface 3 as on the physical controller. Audio needs high speed.
func withAudio(desc usb.Descriptor) usb.Descriptor {
	hid := desc.Interfaces[0]
	hid.Descriptor.BInterfaceNumber = InterfaceHIDAudio
	hid.Endpoints = append([]usb.EndpointDescriptor(nil), hid.Endpoints...)
	for i := range hid.Endpoints {
		// High-speed interrupt intervals count 2^(n-1) microframes: 4 is 1 ms.
		hid.Endpoints[i].BInterval = 4
	}

	desc.Device.Speed = 3
	desc.Configuration.BMAttributes = 0xC0
	desc.Configuration.BMaxPower = 0xFA
	desc.Interfaces = []usb.InterfaceConfig{
		{
			Descriptor: usb.InterfaceDescriptor{
				BInterfaceNumber:   InterfaceAudioControl,
				BInterfaceClass:    0x01, // Audio
				BInterfaceSubClass: 0x01, // AudioControl
			},
			ClassDescriptors: []usb.ClassSpecificDescriptor{
				// Header: UAC 1.00, total length 0x49, streaming interfaces 1 and 2.
				{DescriptorType: 0x24, Payload: usb.Data{0x01, 0x00, 0x01, 0x49, 0x00, 0x02, 0x01, 0x02}},
				// Input terminal 1: USB streaming, 4 channels (L, R, Ls, Rs).
				{DescriptorType: 0x24, Payload: usb.Data{0x02, 0x01, 0x01, 0x01, 0x06, 0x04, 0x33, 0x00, 0x00, 0x00}},
				// Feature unit 2 on terminal 1: master mute and volume.
				{DescriptorType: 0x24, Payload: usb.Data{0x06, 0x02, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}},
				// Output terminal 3: speaker, from unit 2.
				{DescriptorType: 0x24, Payload: usb.Data{0x03, 0x03, 0x01, 0x03, 0x04, 0x02, 0x00}},
				// Input terminal 4: microphone, 2 channels.
				{DescriptorType: 0x24, Payload: usb.Data{0x02, 0x04, 0x02, 0x04, 0x03, 0x02, 0x03, 0x00, 0x00, 0x00}},
				// Feature unit 5 on terminal 4: master mute and volume.
				{DescriptorType: 0x24, Payload: usb.Data{0x06, 0x05, 0x04, 0x01, 0x03, 0x00, 0x00}},
				// Output terminal 6: USB streaming, from unit 5.
				{DescriptorType: 0x24, Payload: usb.Data{0x03, 0x06, 0x01, 0x01, 0x01, 0x05, 0x00}},
			},
		},
		{
			Descriptor: usb.InterfaceDescriptor{BInterfaceNumber: InterfaceHapticsAudio, BInterfaceClass: 0x01, BInterfaceSubClass: 0x02},
		},
		{
			Descriptor: usb.InterfaceDescriptor{BInterfaceNumber: InterfaceHapticsAudio, BAlternateSetting: 1, BNumEndpoints: 1, BInterfaceClass: 0x01, BInterfaceSubClass: 0x02},
			ClassDescriptors: []usb.ClassSpecificDescriptor{
				{DescriptorType: 0x24, Payload: usb.Data{0x01, 0x01, 0x01, 0x01, 0x00}},
				{DescriptorType: 0x24, Payload: usb.Data{0x02, 0x01, SpeakerChannels, audioBytesPerSample, 16, 0x01, 0x80, 0xBB, 0x00}},
			},
			Endpoints: []usb.EndpointDescriptor{{
				BEndpointAddress: EndpointHapticsAudioOut,
				BMAttributes:     0x09, // isochronous, adaptive
				WMaxPacketSize:   speakerMaxPacketSize,
				BInterval:        4,
				Trailing:         usb.Data{0x00, 0x00},
				ClassDescriptors: []usb.ClassSpecificDescriptor{{DescriptorType: 0x25, Payload: usb.Data{0x01, 0x00, 0x00, 0x00, 0x00}}},
			}},
		},
		{
			Descriptor: usb.InterfaceDescriptor{BInterfaceNumber: InterfaceMicrophone, BInterfaceClass: 0x01, BInterfaceSubClass: 0x02},
		},
		{
			Descriptor: usb.InterfaceDescriptor{BInterfaceNumber: InterfaceMicrophone, BAlternateSetting: 1, BNumEndpoints: 1, BInterfaceClass: 0x01, BInterfaceSubClass: 0x02},
			ClassDescriptors: []usb.ClassSpecificDescriptor{
				{DescriptorType: 0x24, Payload: usb.Data{0x01, 0x06, 0x01, 0x01, 0x00}},
				{DescriptorType: 0x24, Payload: usb.Data{0x02, 0x01, MicrophoneChannels, audioBytesPerSample, 16, 0x01, 0x80, 0xBB, 0x00}},
			},
			Endpoints: []usb.EndpointDescriptor{{
				BEndpointAddress: EndpointMicrophoneIn,
				BMAttributes:     0x05, // isochronous, asynchronous
				WMaxPacketSize:   microphoneMaxPacketSize,
				BInterval:        4,
				Trailing:         usb.Data{0x00, 0x00},
				ClassDescriptors: []usb.ClassSpecificDescriptor{{DescriptorType: 0x25, Payload: usb.Data{0x01, 0x00, 0x00, 0x00, 0x00}}},
			}},
		},
		hid,
	}
	return desc
}

// HandleIsoOut takes the speaker stream and passes its haptics channels on.
func (d *DualSense) HandleIsoOut(ep uint32, packets [][]byte) {
	if d.audio == nil || ep != EndpointHapticsAudioOut {
		return
	}
	const frameSize = SpeakerChannels * audioBytesPerSample

	d.audio.mtx.Lock()
	defer d.audio.mtx.Unlock()
	if d.audio.hapticsFunc == nil {
		return
	}
	samples := d.audio.scratch[:0]
	for _, p := range packets {
		for i := 0; i+frameSize <= len(p); i += frameSize {
			// Channels 3 and 4 (Ls, Rs) are the left and right actuators.
			samples = append(samples,
				int16(binary.LittleEndian.Uint16(p[i+4:])),
				int16(binary.LittleEndian.Uint16(p[i+6:])))
		}
	}
	d.audio.scratch = samples
	if len(samples) > 0 {
		d.audio.hapticsFunc(samples)
	}
}

// HandleIsoIn answers the microphone with silence.
func (d *DualSense) HandleIsoIn(ep uint32, packets [][]byte) {
	for _, p := range packets {
		clear(p)
	}
}

// handleAudioControl answers the standard interface requests for the audio
// interfaces' alternate settings and the USB audio class requests.
func (d *DualSense) handleAudioControl(bm, req uint8, wValue, wIndex, wLength uint16, data []byte) ([]byte, bool) {
	a := d.audio
	a.mtx.Lock()
	defer a.mtx.Unlock()

	switch {
	case bm == 0x01 && req == usbRequestSetInterface:
		a.alt[wIndex] = uint8(wValue)
		return nil, true
	case bm == 0x81 && req == usbRequestGetInterface:
		return []byte{a.alt[wIndex]}, true
	}

	switch bm {
	case 0x22: // class, endpoint, host to device: sampling frequency
		if req == audioRequestSetCurrent && len(data) >= 3 {
			a.sampleRate[wIndex] = append([]byte(nil), data[:3]...)
		}
		return nil, true
	case 0xA2: // class, endpoint, device to host
		if rate, ok := a.sampleRate[wIndex]; ok {
			return rate, true
		}
		return []byte{0x80, 0xBB, 0x00}, true
	case 0x21: // class, interface, host to device: feature unit controls
		if uint8(wIndex) != InterfaceAudioControl {
			return nil, false
		}
		unit := wIndex >> 8
		switch uint8(wValue >> 8) {
		case audioControlMute:
			if len(data) >= 1 {
				a.mute[unit] = data[0]
			}
		case audioControlVolume:
			if len(data) >= 2 {
				a.volume[unit] = int16(binary.LittleEndian.Uint16(data))
			}
		}
		return nil, true
	case 0xA1: // class, interface, device to host
		if uint8(wIndex) != InterfaceAudioControl {
			return nil, false
		}
		unit := wIndex >> 8
		switch uint8(wValue >> 8) {
		case audioControlMute:
			return []byte{a.mute[unit]}, true
		case audioControlVolume:
			// Q8.8 dB: -100..0 dB in 1 dB steps, starting at 0 dB.
			var v int16
			switch req {
			case audioRequestGetCurrent:
				v = a.volume[unit]
			case audioRequestGetMinimum:
				v = -100 * 256
			case audioRequestGetMaximum:
				v = 0
			case audioRequestGetResolution:
				v = 256
			}
			return binary.LittleEndian.AppendUint16(nil, uint16(v)), true
		}
		return nil, true
	}
	return nil, false
}
