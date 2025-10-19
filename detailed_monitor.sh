#!/bin/bash

# 詳細な人物検出ノードパフォーマンス監視スクリプト

echo "=== 詳細な人物検出ノード パフォーマンス監視 ==="
echo "監視開始時刻: $(date)"
echo ""

# プロセスIDを取得
PID=$(ps aux | grep "person_detector_node.py" | grep -v grep | awk '{print $2}')

if [ -z "$PID" ]; then
    echo "人物検出ノードが見つかりません"
    exit 1
fi

echo "監視対象プロセスID: $PID"
echo ""

# メインループ
while true; do
    echo "=== $(date) ==="
    
    # プロセスの詳細情報
    echo "プロセス詳細:"
    ps -p $PID -o pid,ppid,cmd,%cpu,%mem,rss,vsz,etime
    
    # CPU使用率の詳細
    echo ""
    echo "CPU使用率詳細:"
    top -p $PID -bn1 | tail -1
    
    # メモリ使用量の詳細
    echo ""
    echo "メモリ使用量詳細:"
    cat /proc/$PID/status | grep -E "(VmSize|VmRSS|VmPeak)"
    
    # システム全体の負荷
    echo ""
    echo "システム全体負荷:"
    echo "CPU使用率: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
    echo "メモリ使用率: $(free | grep Mem | awk '{printf "%.1f%%\n", $3/$2 * 100.0}')"
    
    # プロセス実行時間
    echo ""
    echo "プロセス実行時間:"
    ps -p $PID -o etime
    
    echo ""
    echo "----------------------------------------"
    sleep 10  # 10秒間隔で更新
done 