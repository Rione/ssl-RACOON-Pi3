//go:build rock5a

package upgrade

func boardName() string {
	return "rock5a"
}

func assetFilters() []string {
	return []string{`^racoon-pi2-rock5a_`}
}

func archiveBinaryName() string {
	return "racoon-pi2-rock5a"
}
