#!/usr/bin/env bash
# Install signed Debian ARM toolchain packages locally, without root privileges.
# TLS, archive signatures and package checksums are verified by APT.
set -euo pipefail

toolchain_root=${TOOLCHAIN_ROOT:-/workspace/toolchains}
apt_root="$toolchain_root/apt"
mkdir -p "$apt_root"/{empty,lists/partial,cache/archives/partial,debs}
cat > "$apt_root/apt.conf" <<EOF
Dir::Etc::main "-";
Dir::Etc::parts "$apt_root/empty";
Dir::Etc::sourcelist "$apt_root/sources.list";
Dir::Etc::sourceparts "$apt_root/empty";
Dir::State::lists "$apt_root/lists";
Dir::Cache "$apt_root/cache";
APT::Get::List-Cleanup "false";
EOF
cat > "$apt_root/sources.list" <<'EOF'
deb [signed-by=/usr/share/keyrings/debian-archive-keyring.gpg] https://deb.debian.org/debian trixie main
EOF
export APT_CONFIG="$apt_root/apt.conf"
proxy_options=()
if [[ -n ${HTTPS_PROXY:-} ]]; then
  proxy_options=(-o "Acquire::https::Proxy=$HTTPS_PROXY")
fi
/usr/bin/apt-get "${proxy_options[@]}" update
cd "$apt_root/debs"
/usr/bin/apt-get "${proxy_options[@]}" download \
  'gcc-arm-none-eabi=15:14.2.rel1-1' \
  'binutils-arm-none-eabi=2.44-3+23+b1' \
  'libnewlib-arm-none-eabi=4.5.0.20241231-1' \
  'libnewlib-dev=4.5.0.20241231-1'
mkdir -p "$toolchain_root/arm-debian"
for package in ./*.deb; do
  dpkg-deb -x "$package" "$toolchain_root/arm-debian"
done
"$toolchain_root/arm-debian/usr/bin/arm-none-eabi-gcc" --version
