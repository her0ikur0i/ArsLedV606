#!/bin/bash
# ==============================================================================
# SCRIPT OTOMASI VCS (UNTUK WORKFLOW UPDATE)
# - Menjalankan git add, pio clean, pio build/run.
# - Jika build SUKSES: Merekam perubahan dengan git commit.
# - Jika build GAGAL: Mengembalikan (reset) semua file ke commit terakhir.
# ==============================================================================

# Exit jika ada perintah yang gagal
set -euo pipefail

# Ambil pesan commit dari argumen baris perintah. Default: "VCS Update (Automatic)"
COMMIT_MSG="${1:-VCS Update: New code applied and successfully compiled.}"

# --- 1. Staging Perubahan Terbaru ---
echo "--- 1. Staging Perubahan File Terbaru ---"
git add .
echo "Perubahan telah di-staging (git add .)."

# --- 2. Membersihkan Cache & Kompilasi ---
# Jalankan clean, lalu run (build).
# Jika pio run GAGAL (exit code != 0), maka akan melompat ke blok ELSE (reset).
echo "--- 2. Membersihkan & Kompilasi (Clean Build) ---"
if pio run -t clean && pio run; then
    
    # --- 3. KOMPILASI SUKSES: Commit ---
    echo "--- ✅ KOMPILASI SUKSES: Merekam Perubahan ke Git ---"
    git commit -m "$COMMIT_MSG"
    echo "=========================================================="
    echo "✅ SUKSES: Kode BARU telah dikomit ke Git."
    echo "Pesan: $COMMIT_MSG"
    echo "=========================================================="
else
    # --- 4. KOMPILASI GAGAL: Reset ---
    echo "--- ❌ KOMPILASI GAGAL: Mengembalikan Kode ke Versi Stabil Sebelumnya ---"
    git reset --hard
    echo "=========================================================="
    echo "⚠️ PERINGATAN: Source code telah dikembalikan (reset) ke commit terakhir."
    echo "Silakan cek log kompilasi di atas untuk mengetahui penyebab kegagalan."
    echo "=========================================================="
    exit 1 # Keluar dengan kode error
fi