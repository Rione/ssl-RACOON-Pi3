//go:build linux

package api

import (
	"os/exec"
	"syscall"
)

func configurePythonCmd(cmd *exec.Cmd) {
	cmd.SysProcAttr = &syscall.SysProcAttr{
		// Release the camera if the Go parent exits unexpectedly (Ctrl+C, etc.).
		Pdeathsig: syscall.SIGTERM,
	}
}
