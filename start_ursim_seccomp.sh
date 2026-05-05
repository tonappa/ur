#!/bin/bash
# URSim e-series (ur5e) con seccomp=unconfined + URCap External Control + persistenza.
# Workaround per kernel >= 6.15 (URControl ENOSYS su socket()).

set -e

PERSISTENT_BASE="$HOME/.ursim/e-series"
URCAP_STORAGE="$PERSISTENT_BASE/urcaps"
PROGRAM_STORAGE="$PERSISTENT_BASE/ur5e/programs"
POLYSCOPE_STORAGE="$PERSISTENT_BASE/ur5e/polyscope"

URCAP_VERSION="1.0.5"
URCAP_FILE="$URCAP_STORAGE/externalcontrol-${URCAP_VERSION}.jar"
URCAP_URL="https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar"

mkdir -p "$URCAP_STORAGE" "$PROGRAM_STORAGE" "$POLYSCOPE_STORAGE"

# Rimuovi vecchio .urcap (formato legacy non riconosciuto da PolyScope 5.25)
rm -f "$URCAP_STORAGE"/externalcontrol-*.urcap

if [ ! -f "$URCAP_FILE" ]; then
  echo ">>> Scarico External Control URCap v${URCAP_VERSION}"
  curl -L -o "$URCAP_FILE" "$URCAP_URL"
fi

docker rm -f ursim 2>/dev/null || true
docker network create --subnet=192.168.56.0/24 ursim_net 2>/dev/null || true

docker run --rm -it \
  --name ursim \
  --net ursim_net --ip 192.168.56.101 \
  --security-opt seccomp=unconfined \
  -p 5900:5900 -p 6080:6080 -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -v "$URCAP_STORAGE":/urcaps \
  -v "$PROGRAM_STORAGE":/ursim/programs \
  -v "$POLYSCOPE_STORAGE":/ursim/.polyscope \
  -e ROBOT_MODEL=UR5 \
  universalrobots/ursim_e-series:5.25.1
