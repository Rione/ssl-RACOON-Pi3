//go:build !pi4 && !rock5a

package api

// cameraBoard fallback for tooling/tagless analysis. Real builds always set a
// board tag (pi4 or rock5a); see README "ビルド".
const cameraBoard = "pi4"
