目前使用[UC Agent](https://open-verify.cc/mlvp/docs/ucagent/introduce/)测试

> picker export uart_mmio.v --rw 1 --sname uart_mmio --tdir output/ -c -w output/uart_mmio.fst

> ucagent output/ uart_mmio -s -hm --tui --mcp-server-no-file-tools --no-embed-tools

> verilator --lint-only -Wall --top-module uart_mmio uart_mmio.v
