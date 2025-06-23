#!/bin/bash

# 人物検出ノードのパフォーマンス監視スクリプト

echo "=== 人物検出ノード パフォーマンス監視 ==="
echo "監視開始時刻: $(date)"
echo ""

# メインループ
while true; do
    echo "=== $(date) ==="
    
    # 全体のCPU使用率
    echo "CPU使用率:"
    top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1
    
    # 全体のメモリ使用率
    echo "メモリ使用率:"
    free | grep Mem | awk '{printf "%.1f%%\n", $3/$2 * 100.0}'
    
    # 人物検出ノードのプロセス情報
    echo ""
    echo "人物検出ノードのプロセス情報:"
    ps aux | grep "person_detector_node" | grep -v grep | while read line; do
        if [ ! -z "$line" ]; then
            echo "$line"
        fi
    done
    
    # PythonプロセスのCPU/メモリ使用率
    echo ""
    echo "Pythonプロセスの詳細:"
    ps aux | grep python | grep -v grep | while read line; do
        if [ ! -z "$line" ]; then
            echo "$line"
        fi
    done
    
    echo ""
    echo "----------------------------------------"
    sleep 5  # 5秒間隔で更新
done 