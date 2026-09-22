//go:build !linux

package api

import "os/exec"

func configurePythonCmd(cmd *exec.Cmd) {}
