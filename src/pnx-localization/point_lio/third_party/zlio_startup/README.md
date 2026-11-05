# ZLIO startup subset

Vendored from the local z-localization checkout at the revision recorded in
UPSTREAM.json. The copied sources retain their GPL-2.0-only license (LICENSE).
Only the initialization solver and its pure C++ dependencies are built. No ROS
node, running ZLIO filter, or localization backend is included in this target.
The Point-LIO adapter is outside this directory. Upstream files are unmodified;
UPSTREAM.json records their hashes for review and reproducible updates.
