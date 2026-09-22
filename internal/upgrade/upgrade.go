//go:build pi4 || rock5a

package upgrade

import (
	"archive/tar"
	"bytes"
	"compress/gzip"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"runtime/debug"
	"strings"

	"github.com/blang/semver"
	"github.com/inconshreveable/go-update"
	"github.com/joho/godotenv"
	"github.com/rhysd/go-github-selfupdate/selfupdate"
)

var Version string

const githubRepo = "Rione/ssl-RACOON-Pi3"

// minReleaseBinarySize rejects truncated or mis-packaged release assets before
// they can overwrite the running executable (e.g. empty wget downloads).
const minReleaseBinarySize = 1 << 20 // 1 MiB

// GetVersion returns the current build version string (ldflags-injected on
// release builds, otherwise resolved from build info). Exported so other
// packages (e.g. the PiToMw status sender) can report it to RAVEN.
func GetVersion() string {
	return getVersion()
}

func getVersion() string {
	if Version != "" {
		return Version
	}

	buildInfo, ok := debug.ReadBuildInfo()
	if !ok {
		return "unknown"
	}

	log.Println("Build version:", buildInfo.Main.Version)
	return buildInfo.Main.Version
}

func normalizeVersion(v string) string {
	return strings.TrimPrefix(v, "v")
}

func isDevVersion(v string) bool {
	switch v {
	case "", "(devel)", "unknown":
		return true
	}
	// go build 由来の pseudo-version (例: v1.0.1-0.20260623143125-eab18fdc67ec)
	return strings.Contains(normalizeVersion(v), "-0.")
}

func parseCurrentVersion(v string) (semver.Version, bool) {
	parsed, err := semver.Parse(normalizeVersion(v))
	if err != nil {
		log.Printf("Skipping self-update: cannot parse version %q: %v", v, err)
		return semver.Version{}, false
	}
	return parsed, true
}

func ConfirmAndSelfUpdate() {
	currentVersion := getVersion()
	filters := assetFilters()
	log.Printf("Self-update target: %s (asset filter: %v)", boardName(), filters)

	if isDevVersion(currentVersion) {
		log.Println("NO VERSION INFO (DEV VERSION)")
		return
	}

	_ = godotenv.Load()

	updater, err := selfupdate.NewUpdater(selfupdate.Config{
		APIToken: os.Getenv("GITHUB_TOKEN"),
		Filters:  filters,
	})
	if err != nil {
		log.Println("Error occurred while creating the updater:", err)
		return
	}

	latest, found, err := updater.DetectLatest(githubRepo)
	if err != nil {
		log.Println("Error occurred while detecting version:", err)
		return
	}

	if !found {
		log.Println("No releases found")
		return
	}

	currentSemVer, ok := parseCurrentVersion(currentVersion)
	if !ok {
		return
	}
	if latest.Version.Equals(currentSemVer) || !latest.Version.GT(currentSemVer) {
		log.Println("Current version is the latest")
		return
	}

	log.Println("New version available:", latest.Version)

	cmdPath, err := os.Executable()
	if err != nil {
		log.Println("Error occurred while resolving executable path:", err)
		return
	}

	if err = applyRelease(latest, cmdPath); err != nil {
		log.Println("Error occurred while updating binary:", err)
		return
	}

	log.Println("Successfully updated to version", latest.Version)
	os.Exit(1)
}

func archiveBinaryNames(cmdPath string) []string {
	seen := make(map[string]bool)
	var names []string
	add := func(name string) {
		if name == "" || seen[name] {
			return
		}
		seen[name] = true
		names = append(names, name)
	}

	add(archiveBinaryName())
	add(filepath.Base(cmdPath))
	add("ssl-RACOON-Pi3")
	add("racoon-pi3")
	return names
}

func fetchReleaseAsset(rel *selfupdate.Release) ([]byte, error) {
	resp, err := http.Get(rel.AssetURL)
	if err != nil {
		return nil, fmt.Errorf("download %s: %w", rel.AssetURL, err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("download %s: status %d", rel.AssetURL, resp.StatusCode)
	}
	return io.ReadAll(resp.Body)
}

func applyRelease(rel *selfupdate.Release, targetPath string) error {
	data, err := fetchReleaseAsset(rel)
	if err != nil {
		return err
	}

	if err := applyBinary(data, rel.AssetURL, targetPath); err != nil {
		return err
	}

	// The binary is already updated; treat camera/model extraction as
	// best-effort so a packaging issue cannot leave us with a half-applied
	// update that refuses to start.
	destDir := filepath.Dir(targetPath)
	if err := extractCameraFiles(data, destDir); err != nil {
		log.Printf("Self-update: binary updated but camera/model extraction failed: %v", err)
	}
	return nil
}

func validateReleaseBinary(payload []byte) error {
	if len(payload) < minReleaseBinarySize {
		return fmt.Errorf(
			"release binary too small: %d bytes (minimum %d)",
			len(payload), minReleaseBinarySize,
		)
	}
	if len(payload) < 4 || payload[0] != 0x7f || payload[1] != 'E' || payload[2] != 'L' || payload[3] != 'F' {
		return fmt.Errorf("release binary is not an ELF executable")
	}
	return nil
}

func applyBinary(data []byte, assetURL, targetPath string) error {
	var lastErr error
	for _, name := range archiveBinaryNames(targetPath) {
		asset, err := selfupdate.UncompressCommand(bytes.NewReader(data), assetURL, name)
		if err != nil {
			lastErr = err
			continue
		}
		payload, err := io.ReadAll(asset)
		if err != nil {
			lastErr = fmt.Errorf("read archive binary %q: %w", name, err)
			continue
		}
		if err := validateReleaseBinary(payload); err != nil {
			lastErr = fmt.Errorf("archive binary %q: %w", name, err)
			log.Printf("Self-update: rejecting %q from %s: %v", name, assetURL, err)
			continue
		}
		log.Printf("Updating %s using archive binary %q (%d bytes)", targetPath, name, len(payload))
		if err := update.Apply(bytes.NewReader(payload), update.Options{TargetPath: targetPath}); err != nil {
			return err
		}
		return nil
	}
	if lastErr != nil {
		return lastErr
	}
	return fmt.Errorf("no archive binary name candidates for %s", targetPath)
}

// extractCameraFiles extracts the camera/ Python tree from the release tarball
// into destDir so self-update keeps the camera package in sync with the binary.
// YOLO .pt weights are published as a separate release asset and are not
// re-written on self-update (see skipCameraExtract).
func extractCameraFiles(data []byte, destDir string) error {
	gz, err := gzip.NewReader(bytes.NewReader(data))
	if err != nil {
		return fmt.Errorf("open gzip: %w", err)
	}
	defer gz.Close()

	tr := tar.NewReader(gz)
	var count int
	for {
		hdr, err := tr.Next()
		if err == io.EOF {
			break
		}
		if err != nil {
			return fmt.Errorf("read tar: %w", err)
		}
		if hdr.Typeflag != tar.TypeReg {
			continue
		}

		rel := filepath.Clean(hdr.Name)
		if !underCameraDir(rel) {
			continue
		}

		outPath := filepath.Join(destDir, rel)
		// Guard against path traversal from a malformed archive.
		if !strings.HasPrefix(outPath, filepath.Clean(destDir)+string(os.PathSeparator)) {
			log.Printf("Self-update: skipping suspicious archive path %q", hdr.Name)
			continue
		}
		if skipCameraExtract(rel, outPath) {
			if _, err := io.Copy(io.Discard, tr); err != nil {
				return fmt.Errorf("skip %s: %w", rel, err)
			}
			continue
		}
		if err := writeFileAtomic(outPath, tr, os.FileMode(hdr.Mode)); err != nil {
			return fmt.Errorf("write %s: %w", rel, err)
		}
		count++
	}

	log.Printf("Self-update: refreshed %d camera file(s) under %s", count, destDir)
	return nil
}

// skipCameraExtract returns true when an archive entry should not be written.
// YOLO weights are large and rarely change; keep the local copy and avoid
// re-transferring them on every self-update (older releases may still ship .pt).
func skipCameraExtract(rel, outPath string) bool {
	if !strings.HasSuffix(strings.ToLower(rel), ".pt") {
		return false
	}
	if _, err := os.Stat(outPath); err == nil {
		log.Printf("Self-update: keeping existing model %s", rel)
		return true
	}
	return false
}

func underCameraDir(rel string) bool {
	return rel == "camera" || strings.HasPrefix(rel, "camera"+string(os.PathSeparator))
}

func writeFileAtomic(outPath string, r io.Reader, mode os.FileMode) error {
	if err := os.MkdirAll(filepath.Dir(outPath), 0o755); err != nil {
		return err
	}
	tmp, err := os.CreateTemp(filepath.Dir(outPath), ".racoon-update-*")
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	defer os.Remove(tmpName)

	if _, err := io.Copy(tmp, r); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	if mode != 0 {
		if err := os.Chmod(tmpName, mode); err != nil {
			return err
		}
	}
	return os.Rename(tmpName, outPath)
}
