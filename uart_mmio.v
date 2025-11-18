`ifndef UART_MMIO_V
`define UART_MMIO_V

`timescale 1ns/1ps

`ifndef UART_DEFAULT_BAUD
`define UART_DEFAULT_BAUD 115200
`endif

`ifndef UART_FIFO_DEPTH
`define UART_FIFO_DEPTH 16
`endif

module uart_mmio #(
    parameter [31:0]  BASE_ADDR     = 32'h8000_1000,
    parameter [31:0]  CLK_FREQ      = 32'd100_000_000,      // oscc default 100MHz
    parameter integer DEFAULT_BAUD  = `UART_DEFAULT_BAUD,
    parameter integer FIFO_DEPTH    = `UART_FIFO_DEPTH,
    parameter integer MIN_BAUD      = 1500
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     mem_valid,
    input  wire                     mem_instr,
    output reg                      mem_ready,
    input  wire [31:0]              mem_addr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,
    output reg  [31:0]              mem_rdata,

    input  wire                     uart_rx,
    output reg                      uart_tx,

    input  wire                     uart_cts,
    output reg                      uart_rts,

    output reg                      irq,
    input  wire                     eoi                     // end of interrupt: clear irq
);
    reg irq_next;

    reg [31:0] ctrl_reg, baud_reg, baud_cnt;
    /* verilator lint_off UNUSEDSIGNAL */
    reg [31:0] status_regs;
    /* verilator lint_on  UNUSEDSIGNAL */
    reg [7:0] tx_fifo [0:FIFO_DEPTH-1];
    reg [7:0] rx_fifo [0:FIFO_DEPTH-1];

    reg [$clog2(FIFO_DEPTH)-1:0] tx_sd_ptr, tx_wr_ptr;
    reg [$clog2(FIFO_DEPTH)-1:0] rx_rd_ptr, rx_sv_ptr;
    reg [$clog2(FIFO_DEPTH)  :0] tx_count , rx_count ;

    reg [7:0]   tx_shift;
    reg [3:0]   tx_bit_cnt;
    reg         tx_parity;
    reg         tx_active;
    reg [1:0]   tx_stop_cnt;

    reg [3:0]   rx_os_cnt;
    reg [7:0]   rx_shift;
    reg [3:0]   rx_bit_cnt;
    reg         rx_active;
    reg         rx_parity;
    reg [1:0]   rx_stop_cnt;
    reg         uart_rx_sync, uart_rx_prev;

    // for loopback mode
    reg         tx_logical;
    reg         rx_logical;

    wire baud_tick = (baud_cnt == 0);
    wire rx_is_empty = (rx_count == 0);

    wire [$clog2(FIFO_DEPTH):0] tx_count_inc;
    wire [31:0] wmask = { {8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}} };
    wire [31:0] wdata = mem_wdata & wmask;

    localparam [31:0]
        UART_TX             = BASE_ADDR + 32'h00,
        UART_RX             = BASE_ADDR + 32'h04,
        UART_STATUS         = BASE_ADDR + 32'h08,
        UART_CTRL           = BASE_ADDR + 32'h0C,
        UART_BAUD           = BASE_ADDR + 32'h10,
        UART_BAUD_MIN       = BASE_ADDR + 32'h14,
        UART_BAUD_MAX       = BASE_ADDR + 32'h18;

    localparam [31:0] MAX_BAUD = CLK_FREQ;

    wire ctrl_rx_irq_en       = ctrl_reg[0];        // RX IRQ enable
    wire ctrl_tx_irq_en       = ctrl_reg[1];        // TX IRQ enable
    wire ctrl_uart_en         = ctrl_reg[2];        // UART enable
    wire ctrl_parity_en       = ctrl_reg[3];        // parity enable
    wire ctrl_even_parity_sel = ctrl_reg[4];        // even parity select
    wire ctrl_two_stop_bits   = ctrl_reg[5];        // 2 stop bits select
    wire ctrl_rts_cts_en      = ctrl_reg[6];        // RTS/CTS flow control
    wire ctrl_loopback        = ctrl_reg[7];        // loopback mode

    function [31:0] zext32;
        input [$clog2(FIFO_DEPTH):0] in;
        begin
            zext32 = 32'b0;
            zext32[$clog2(FIFO_DEPTH):0] = in;
        end
    endfunction

    function [31:0] zext32_8;
        input [7:0] in;
        begin
            zext32_8 = {24'b0, in};
        end
    endfunction

    // status about regs: bit 0-6
    wire status_rx_ready      = (zext32(rx_count) != 0);
    wire status_tx_ready      = (zext32(tx_count) != FIFO_DEPTH);
    wire status_rx_overrun    = status_regs[2];
    wire status_frame_error   = status_regs[3];
    wire status_parity_error  = status_regs[4];
    wire status_tx_empty      = (zext32(tx_count) == 0);
    wire status_rx_full       = (zext32(rx_count) == FIFO_DEPTH);
    wire status_rx_empty      = status_regs[7];

    wire [31:0] status_wire = {
        24'b0,
        status_rx_empty,
        status_rx_full,
        status_tx_empty,
        status_parity_error,
        status_frame_error,
        status_rx_overrun,
        status_tx_ready,
        status_rx_ready
    };

    always @(posedge clk) begin: OSC
        if (!resetn)
            baud_cnt <= 0;
        else if (ctrl_uart_en) begin
            if (baud_cnt == 0)
                baud_cnt <= (baud_reg == 0 ? 1 : baud_reg);
            else
                baud_cnt <= baud_cnt - 1;
        end else ;
    end

    always @(*) begin: LOOPBACK_MODE
        if (ctrl_loopback) begin
            uart_tx = 1'b1;
            rx_logical = tx_logical;
        end else begin
            uart_tx = tx_logical;
            rx_logical = uart_rx;
        end
    end

    // begin send and mmio write txfifo in the same time
    assign tx_count_inc = (mem_valid && !mem_instr && mem_addr == UART_TX && (zext32(tx_count) < FIFO_DEPTH)) ? 1 : 0;

    always @(posedge clk) begin: TX_SEND_DATA
        if (!resetn) begin
            tx_logical  <= 1'b1;
            tx_active   <= 0;
            tx_bit_cnt  <= 0;
            tx_stop_cnt <= 0;
            tx_sd_ptr   <= 0;
            tx_count    <= 0;
        end else if (ctrl_uart_en) begin
            if (!tx_active && tx_count > 0 && (!ctrl_rts_cts_en || uart_cts)) begin: NEED_SEND_AND_BEGIN_SEND
                tx_shift <= tx_fifo[tx_sd_ptr];
                tx_sd_ptr <= tx_sd_ptr + 1;
                tx_count <= tx_count - 1 + tx_count_inc;
                if (ctrl_parity_en)
                    tx_parity <= ctrl_even_parity_sel ? ~(^tx_fifo[tx_sd_ptr]) : (^tx_fifo[tx_sd_ptr]);
                else
                    tx_parity <= 1'b0;
                tx_bit_cnt <= 0; tx_stop_cnt <= 0; tx_active <= 1'b1; tx_logical <= 1'b0;
            end else tx_count <= tx_count + tx_count_inc;
            if (tx_active && baud_tick) begin
                if (tx_bit_cnt < 4'd8) begin: TX_SEND_BITS_UNTILE_8
                    tx_logical <= tx_shift[tx_bit_cnt[2:0]];
                    tx_bit_cnt <= tx_bit_cnt + 1;
                end else if (tx_bit_cnt == 4'd8 && ctrl_parity_en) begin: TX_SENT_PARITY
                    tx_logical <= tx_parity;
                    tx_bit_cnt <= tx_bit_cnt + 1;
                end else begin: TX_SEND_STOP_BITS
                    tx_logical <= 1'b1;
                    if (ctrl_two_stop_bits) begin
                        if (tx_stop_cnt == 1) tx_active <= 0;
                        tx_stop_cnt <= tx_stop_cnt + 1;
                    end else tx_active <= 0;
                end
            end else ;
        end else ;
    end

    always @(posedge clk) begin: RX_SAVE_DATA
        if (!resetn) begin
            uart_rx_sync   <= 1'b1;
            uart_rx_prev   <= 1'b1;
            rx_active      <= 0;
            rx_count       <= 0;
            rx_sv_ptr      <= 0;
            status_regs[2] <= 0;
            status_regs[3] <= 0;
            status_regs[4] <= 0;
            rx_stop_cnt    <= 0;
        end else if (ctrl_uart_en) begin
            uart_rx_sync <= rx_logical;
            uart_rx_prev <= uart_rx_sync;
            if (~rx_active && uart_rx_prev & ~uart_rx_sync) begin: NEED_TO_RECIVE_DATA_AND_START_RECV
                rx_active   <= 1'b1;
                rx_os_cnt   <= 0;
                rx_bit_cnt  <= 0;
                rx_stop_cnt <= 0;
            end else if (rx_active && baud_tick) begin
                if (rx_os_cnt == 15) begin: X16_SAMPLE
                    if (rx_bit_cnt < 4'd8) begin: RECV_UNTILE_8_BITS
                        rx_shift[rx_bit_cnt[2:0]] <= uart_rx_sync;
                        rx_bit_cnt <= rx_bit_cnt + 1;
                    end else if (rx_bit_cnt == 4'd8 && ctrl_parity_en) begin: RX_BEGIN_CHECK_PARITY
                        rx_parity <= uart_rx_sync;
                        if (ctrl_even_parity_sel ? ~(^rx_shift) != rx_parity : (^rx_shift) != rx_parity)
                            status_regs[4] <= 1;  // rx parity error
                        else ;
                        rx_bit_cnt <= rx_bit_cnt + 1;
                    end else begin: RX_STOP_BITS
                        if (uart_rx_sync != 1'b1) status_regs[3] <= 1; else ;  // rx frame error
                        rx_stop_cnt <= rx_stop_cnt + 1;
                        if (ctrl_two_stop_bits ? rx_stop_cnt == 1 : rx_stop_cnt == 0) begin: WAIT_STOP_AND_SAVE_BITS
                            rx_active <= 0;
                            if (zext32(rx_count) < FIFO_DEPTH) begin
                                rx_fifo[rx_sv_ptr] <= rx_shift;
                                rx_sv_ptr <= rx_sv_ptr + 1;
                                rx_count <= rx_count + 1;
                            end else begin: RX_FULL
                                status_regs[2] <= 1;  // rx overrun
                            end
                        end else ;
                    end
                    rx_os_cnt <= 0;
                end else rx_os_cnt <= rx_os_cnt + 1;
            end else ;
        end else ;
    end

    always @(*) begin: IRQ
        irq_next = 0;
        if (ctrl_uart_en) begin
            if ((ctrl_rx_irq_en && status_rx_ready) ||
                (ctrl_tx_irq_en && status_tx_ready) ||
                (status_rx_overrun || status_frame_error || status_parity_error)) begin
               irq_next = 1; end else ;
        end else ;
    end

    always @(posedge clk) begin
    if (!resetn)
        irq <= 0;
    else
        irq <= eoi ? 0 : irq_next;
    end

    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            rx_rd_ptr <= 0;
        end
        if (mem_valid && !mem_instr) begin
            case (mem_addr)
                UART_TX:          mem_rdata <= zext32_8(tx_fifo[tx_sd_ptr]);
                UART_STATUS:      mem_rdata <= status_wire;
                UART_CTRL:        mem_rdata <= ctrl_reg;
                UART_BAUD:        mem_rdata <= baud_reg;
                UART_BAUD_MIN:    mem_rdata <= MIN_BAUD;
                UART_BAUD_MAX:    mem_rdata <= MAX_BAUD;
                UART_RX:          begin
                    if (!rx_is_empty) begin
                        mem_rdata <= zext32_8(rx_fifo[rx_rd_ptr]);
                        rx_rd_ptr <= rx_rd_ptr + 1;
                    end else begin
                        mem_rdata <= 0;
                        status_regs[7] <= 1;
                    end
                end
                default:          mem_rdata <= 0;
            endcase
        end else mem_rdata <= 0;
    end

    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            mem_ready <= 0;
            ctrl_reg  <= 0;
            baud_reg  <= CLK_FREQ / DEFAULT_BAUD;
            tx_wr_ptr <= 0;
        end else begin
            if (mem_valid && !mem_instr) begin
                mem_ready <= 1;
                case(mem_addr)
                    UART_TX:
                    if (zext32(tx_count) < FIFO_DEPTH) begin: WRITE_BITS_TO_TX_FIFO
                        tx_fifo[tx_wr_ptr] <= wdata[7:0];
                        tx_wr_ptr <= tx_wr_ptr + 1;
                    end else ;
                    UART_CTRL:  ctrl_reg[7:0] <= wdata[7:0];
                    UART_BAUD:  baud_reg      <= wdata;
                    UART_STATUS: begin
                        if (wdata[2]) status_regs[2] <= 0;
                        if (wdata[3]) status_regs[3] <= 0;
                        if (wdata[4]) status_regs[4] <= 0;
                        if (wdata[7]) status_regs[7] <= 0;
                    end
                    default: ;
                endcase
            end else mem_ready <= 0;
        end
    end

    always @(*) begin: RTS_CTS
        if (ctrl_rts_cts_en)
            uart_rts = (zext32(rx_count) > FIFO_DEPTH/2);
        else
            uart_rts = 1'b0;
    end

endmodule

`endif
