#!/bin/bash
cd "$(dirname "$0")"

# --- 1. 設定身分 ---
git config user.name "tkuiclabwhite"
git config user.email "tkuiclabwhite@gmail.com"

BRANCH="main"

# --- 2. 定義暱稱映射表 ---
declare -A NICKNAMES
NICKNAMES=(
   ["med"]="src/med_detect"
   ["table"]="src/med_detect/med_detect/Parameter"
   ["all"]="."
)

# --- 3. 處理輸入參數 ---
INPUT=$1

if [ -z "$INPUT" ]; then
    TARGET="."
elif [[ -n "${NICKNAMES[$INPUT]}" ]]; then
    TARGET="${NICKNAMES[$INPUT]}"
else
    TARGET="$INPUT"
fi

echo "📂 目標路徑：$TARGET"

if [ ! -d "$TARGET" ] && [ "$TARGET" != "." ]; then
    echo "❌ 錯誤：找不到路徑 '$TARGET'，請檢查暱稱或路徑是否正確。"
    echo "   可用暱稱：${!NICKNAMES[*]}"
    exit 1
fi

# --- 4. 檢查網路是否能連到這個 repo 的 remote ---
echo "🌐 檢查與 GitHub 的連線..."
if ! git ls-remote origin "$BRANCH" &> /dev/null; then
    echo "❌ 無法連接到 GitHub remote (origin)。"
    echo "   可能原因：未連接 WiFi、WiFi 無法上網、或 DNS/防火牆問題。"
    echo "   請確認網路後再執行一次。"
    exit 1
fi
echo "✅ 連線正常"

# --- 5. 加入變動並視情況 commit ---
git add "$TARGET"

if git diff-index --quiet HEAD --; then
    echo "ℹ️  沒有偵測到新變動。"
else
    current_date=$(date +"%Y-%m-%d %H:%M")
    git commit -m "Update ($TARGET): $current_date"
fi

# --- 6. 跟遠端比對，看本地是否領先 ---
git fetch origin "$BRANCH" --quiet

LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse "origin/$BRANCH")

if [ "$LOCAL" = "$REMOTE" ]; then
    echo "✅ 本地與遠端已同步，沒有需要上傳的內容。"
    exit 0
fi

# --- 7. 執行 push 並檢查結果 ---
git push origin "$BRANCH"
PUSH_STATUS=$?

echo "-------------------------------"
if [ $PUSH_STATUS -eq 0 ]; then
    echo "✅ 上傳成功！[push package: ${INPUT:-all}] -> $TARGET"
    echo "✅ 日期: $(date +"%Y-%m-%d %H:%M")"
else
    echo "❌ 上傳失敗！git push 回傳錯誤，請檢查上方訊息。"
    echo "（commit 已存在本機，下次有網路時重跑這支腳本即可補推）"
    exit 1
fi
echo "-------------------------------"
