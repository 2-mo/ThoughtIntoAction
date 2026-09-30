#!/usr/bin/env bash
set -Eeuo pipefail

if (($# != 2)); then
  echo "用法：$0 LAN_IP CERT_DIR" >&2
  exit 2
fi

LAN_IP="$1"
CERT_DIR="$2"
python3 -c 'import ipaddress, sys; ipaddress.ip_address(sys.argv[1])' "$LAN_IP"
command -v openssl >/dev/null 2>&1 || {
  echo "缺少 openssl，无法生成局域网 HTTPS 证书。" >&2
  exit 1
}

umask 077
mkdir -p "$CERT_DIR"
CA_KEY="$CERT_DIR/arx5-local-ca.key"
CA_CERT="$CERT_DIR/arx5-local-ca.crt"
SERVER_KEY="$CERT_DIR/arx5-web.key"
SERVER_CERT="$CERT_DIR/arx5-web.crt"

if [[ ! -s "$CA_KEY" || ! -s "$CA_CERT" ]]; then
  openssl req -x509 -newkey rsa:3072 -sha256 -days 3650 -nodes \
    -subj "/CN=ARX5 Local Web CA" \
    -keyout "$CA_KEY" -out "$CA_CERT" >/dev/null 2>&1
fi

NEED_SERVER_CERT=0
if [[ ! -s "$SERVER_KEY" || ! -s "$SERVER_CERT" ]]; then
  NEED_SERVER_CERT=1
elif ! openssl x509 -in "$SERVER_CERT" -noout -checkend 86400 >/dev/null 2>&1; then
  NEED_SERVER_CERT=1
elif ! openssl x509 -in "$SERVER_CERT" -noout -ext subjectAltName 2>/dev/null \
  | grep -Fq "IP Address:$LAN_IP"; then
  NEED_SERVER_CERT=1
fi

if ((NEED_SERVER_CERT)); then
  CSR_PATH="$(mktemp "$CERT_DIR/arx5-web.XXXXXX.csr")"
  EXT_PATH="$(mktemp "$CERT_DIR/arx5-web.XXXXXX.ext")"
  cleanup_temp() {
    rm -f -- "$CSR_PATH" "$EXT_PATH"
  }
  trap cleanup_temp EXIT
  printf '%s\n' \
    "subjectAltName=IP:$LAN_IP,IP:127.0.0.1,DNS:localhost" \
    "extendedKeyUsage=serverAuth" >"$EXT_PATH"
  openssl req -new -newkey rsa:2048 -sha256 -nodes \
    -subj "/CN=$LAN_IP" \
    -keyout "$SERVER_KEY" -out "$CSR_PATH" >/dev/null 2>&1
  openssl x509 -req -in "$CSR_PATH" -sha256 -days 825 \
    -CA "$CA_CERT" -CAkey "$CA_KEY" -CAcreateserial \
    -extfile "$EXT_PATH" -out "$SERVER_CERT" >/dev/null 2>&1
fi

chmod 600 "$CA_KEY" "$SERVER_KEY"
chmod 644 "$CA_CERT" "$SERVER_CERT"
echo "局域网 HTTPS 证书已就绪：$SERVER_CERT"
