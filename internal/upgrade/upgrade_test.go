//go:build pi4 || rock5a

package upgrade

import (
	"strings"
	"testing"
)

func TestSkipCameraExtract(t *testing.T) {
	if skipCameraExtract("camera/foo.py", "/tmp/unused") {
		t.Fatal("python files should not be skipped")
	}
	if skipCameraExtract("camera/yolo/last.pt", "/nonexistent/path/last.pt") {
		t.Fatal("missing model should be extracted")
	}
}

func TestUnderCameraDir(t *testing.T) {
	if !underCameraDir("camera/main.py") {
		t.Fatal("expected camera/main.py under camera dir")
	}
	if underCameraDir("README.md") {
		t.Fatal("README should not be under camera dir")
	}
}

func TestValidateReleaseBinary(t *testing.T) {
	validELF := make([]byte, minReleaseBinarySize)
	validELF[0], validELF[1], validELF[2], validELF[3] = 0x7f, 'E', 'L', 'F'

	tests := []struct {
		name    string
		payload []byte
		wantErr string
	}{
		{
			name:    "empty",
			payload: nil,
			wantErr: "too small",
		},
		{
			name:    "under 1MiB",
			payload: []byte{0x7f, 'E', 'L', 'F'},
			wantErr: "too small",
		},
		{
			name:    "large but not ELF",
			payload: make([]byte, minReleaseBinarySize),
			wantErr: "not an ELF",
		},
		{
			name:    "valid ELF",
			payload: validELF,
			wantErr: "",
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			err := validateReleaseBinary(tt.payload)
			if tt.wantErr == "" {
				if err != nil {
					t.Fatalf("validateReleaseBinary() error = %v", err)
				}
				return
			}
			if err == nil || !strings.Contains(err.Error(), tt.wantErr) {
				t.Fatalf("validateReleaseBinary() error = %v, want substring %q", err, tt.wantErr)
			}
		})
	}
}
