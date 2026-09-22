//go:build pi4

package upgrade

func boardName() string {
	return "pi4"
}

func assetFilters() []string {
	return []string{`^racoon-pi3-pi4_`}
}

func archiveBinaryName() string {
	return "racoon-pi3-pi4"
}
