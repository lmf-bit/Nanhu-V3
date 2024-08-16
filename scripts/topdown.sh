#!/bin/bash
# ./topdown [log]


cpu_cycle=$(grep clock_cycle $1 | awk '{print $6}')
backend_stall=$(grep Topdown_Backend_Stall $1 | awk '{print $6}')
frontend_stall=$(grep Topdown_Frontend_Stall $1 | awk '{print $6}')
stall=$(grep Topdown_Stall $1 | awk '{print $6}')
op_spec=$(grep Topdown_Op_spec $1 | awk '{print $6}')
op_retired=$(grep commitInstr, $1 | awk '{print $6}')
mis_pred=$(grep Topdown_Mispredict $1 | awk '{print $6}')



echo "cycle count: $cpu_cycle"
echo "backend stall: $backend_stall"
echo "frontend stall: $frontend_stall"
echo "stall: $stall"
echo "op spec: $op_spec"
echo "op retired: $op_retired"
echo "mis pred: $mis_pred"

function calculate() {
  local expression="$1"  # 传递包含运算的字符串
  local result             # 结果变量
  result=$(echo "scale=5; $expression" | bc -l)  # 使用 bc 进行计算，保留两位小数
  echo "$result"          # 输出结果
}

backend_bound=$(calculate   "100 * $backend_stall / ($cpu_cycle * 4)")
bad_speculation=$(calculate "100 * ((1 - $op_retired / $op_spec) * (1 - $stall / ($cpu_cycle * 4)) + (($mis_pred * 4) / $cpu_cycle))" )
frontend_bound=$(calculate  "100 * ($frontend_stall / ($cpu_cycle * 4) - ($mis_pred * 4) / $cpu_cycle)")
retire=$(calculate          "100 * ($op_retired / $op_spec) * (1 - $stall/($cpu_cycle * 4))")

echo "--------topdown counter---------"

echo "- backend bound: $backend_bound"
echo "- bad speculation: $bad_speculation"
echo "- frontend bound: $frontend_bound"
echo "- retire: $retire"
