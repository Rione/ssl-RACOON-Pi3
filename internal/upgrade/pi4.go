//go:build pi4

package upgrade

func boardName() string {
	return "pi4"
}

func assetFilters() []string {
	return []string{`^racoon-pi2-pi4_`}
}

func archiveBinaryName() string {
	return "racoon-pi2-pi4"
}
