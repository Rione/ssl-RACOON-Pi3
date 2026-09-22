//go:build rock5a

package upgrade

func boardName() string {
	return "rock5a"
}

func assetFilters() []string {
	return []string{`^racoon-pi3-rock5a_`}
}

func archiveBinaryName() string {
	return "racoon-pi3-rock5a"
}
