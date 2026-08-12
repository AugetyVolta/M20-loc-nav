# gtsam_vendor

This package builds the repository's pinned GTSAM 4.1.1 source once for all
workspace consumers. The source under `vendor/gtsam` is the copy originally
shipped with PCT planner and corresponds to upstream commit
`69a3a75195b65356d6e56669e5199d325c7962c9`.

Build it through colcon; do not install another GTSAM into `/usr/local` for this
workspace:

```bash
colcon build --packages-select gtsam_vendor
```

Consumers declare a dependency on `gtsam_vendor`, call
`find_package(gtsam_vendor REQUIRED)`, and link the imported `gtsam` target.
Generated libraries are installed under `install/gtsam_vendor` and are not
tracked by Git.
