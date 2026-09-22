//go:build rock5a

package main

import (
	"errors"
	"sync"
	"syscall"
	"unsafe"

	"github.com/Yuzz1e/rock5a-gpio-go"
)

const (
	devMem   = "/dev/mem"
	pageSize = 4096
)

var (
	iocMu    sync.Mutex
	iocFd    int = -1
	iocCache = make(map[uint32][]byte)
)

func readIOCReg(bank, port int) (uint32, error) {
	base, off, ok := gpio.GetPullReg(bank, rune('A'+port), 0)
	if !ok {
		return 0, errors.New("GetPullReg failed")
	}
	return readMMIO(base, off)
}

func decodePull(pin int, reg uint32) (pe, ps bool, label string) {
	lower := reg & 0xFFFF
	pe = (lower & (1 << (2 * pin))) != 0
	ps = (lower & (1 << (2*pin + 1))) != 0
	switch {
	case !pe:
		label = "floating"
	case ps:
		label = "pull-up"
	default:
		label = "pull-down"
	}
	return pe, ps, label
}

func readMMIO(baseAddr, offset uint32) (uint32, error) {
	iocMu.Lock()
	defer iocMu.Unlock()

	if iocFd < 0 {
		fd, err := syscall.Open(devMem, syscall.O_RDWR|syscall.O_SYNC, 0)
		if err != nil {
			return 0, err
		}
		iocFd = fd
	}

	physAddr := baseAddr + offset
	pageStart := physAddr & ^uint32(pageSize-1)

	slice, ok := iocCache[pageStart]
	if !ok {
		mapped, err := syscall.Mmap(iocFd, int64(pageStart), pageSize,
			syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
		if err != nil {
			return 0, err
		}
		iocCache[pageStart] = mapped
		slice = mapped
	}

	regOff := physAddr - pageStart
	if regOff+4 > pageSize {
		return 0, errors.New("register offset out of mapped page range")
	}
	return *(*uint32)(unsafe.Pointer(&slice[regOff])), nil
}
