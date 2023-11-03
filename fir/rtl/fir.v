module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  reg                     awready,
    output  reg                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  reg                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg                     rvalid,
    output  reg [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                     ss_tready, 
    input   wire                     sm_tready, 
    output  reg                     sm_tvalid, 
    output  reg [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                     sm_tlast, 
    
    // bram for tap RAM
    output  reg [3:0]               tap_WE, //tap is placed at bram and corresponds to 0x20 address in axilite
    output  reg                     tap_EN,
    output  reg [(pDATA_WIDTH-1):0] tap_Di,
    output  reg [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg [3:0]               data_WE,
    output  reg                     data_EN,
    output  reg [(pDATA_WIDTH-1):0] data_Di,
    output  reg [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

    localparam STATE_RECEIVE_PARAMETERS = 0;
    localparam STATE_EXECUTE = 1;
    localparam STATE_CLEAN_DATA_IN_BRAM = 2;
    reg [2-1:0] state_r;
    reg [2-1:0] state_w;
    reg [4-1:0] counter_r;
    reg [4-1:0] counter_w;
    reg ap_done, ap_idle; 
    reg [32-1:0] data_length_w;
    reg [32-1:0] data_length_r;
    reg [32-1:0] receive_data_for_write_w; //address or data
    reg [32-1:0] receive_data_for_write_r;
    reg [32-1:0] receive_address_for_read_w;
    reg [32-1:0] receive_address_for_read_r;
    reg [2-1:0] write_state_w; //2'b10 represents address valid, 2'b01 represents data valid
    reg [2-1:0] write_state_r; //2'b10 represents address valid, 2'b01 represents data valid
    reg [32-1:0] fir_result_w;
    reg [32-1:0] fir_result_r;
    reg          bram_wait_w;
    reg          bram_wait_r;
    reg [4-1:0] bram_first_data_position_r; //From 0 to 10
    reg [4-1:0] bram_first_data_position_w; //From 0 to 10
    reg         pending_sm_tready_r;
    reg         pending_sm_tready_w;
    reg         pending_rready_r;
    reg         pending_rready_w;
    reg         last_r;
    reg         last_w;
    
    
    
    always@(*) begin
        receive_address_for_read_w = receive_address_for_read_r;
        receive_data_for_write_w = receive_data_for_write_r;
        counter_w = 0;
        state_w = state_r;
        data_length_w = data_length_r;
        tap_EN = 0;
        tap_WE = 0;
        tap_A = 0;
        tap_Di = 0;
        data_EN = 0;
        data_WE = 0;
        data_A = 0;
        data_Di = 0;
        write_state_w = write_state_r;
        wready = 0;
        awready = 0;
        ss_tready = 0;
        sm_tvalid = 0;
        sm_tdata = 0;
        sm_tlast = 0;
        fir_result_w = fir_result_r;
        bram_wait_w = 1;
        bram_first_data_position_w = bram_first_data_position_r;
        last_w = last_r;
        pending_sm_tready_w = pending_sm_tready_r;
        pending_rready_w = pending_rready_r;
        rvalid = 0;
        rdata = 0;
        arready = 0;
        case (state_r)
            STATE_RECEIVE_PARAMETERS: begin
                if (pending_rready_r == 1'b0) begin //prevent bram from being read and written at the same time
                    case (write_state_r)
                        2'b00: begin //Address and data are both not valid
                            if ((awvalid == 1'b1) & (wvalid == 1'b1)) begin
                                if ((awaddr >> 4) == 0) begin //address = 0x00
                                    if (wdata[0] == 1'b1) begin //ap_start = 1
                                        state_w = STATE_EXECUTE;
                                    end
                                end
                                else if ((awaddr >> 4) == 1) begin //address = 0x10
                                    data_length_w = wdata;
                                end
                                else begin //address = 0x20, direct communicate with BRAM
                                    tap_EN = 1;
                                    tap_WE = 4'b1111;
                                    tap_A = awaddr-'h20;
                                    tap_Di = wdata;
                                    data_EN = 1;
                                    data_WE = 4'b1111;
                                    data_A = awaddr-'h20;
                                    data_Di = 0; //initialization
                                end
                                awready = 1;
                                wready = 1;
                            end
                            else if (awvalid == 1'b1) begin
                                write_state_w = 2'b10;
                                receive_data_for_write_w = awaddr;
                                awready = 1;
                            end
                            else if (wvalid == 1'b1) begin
                                write_state_w = 2'b01;
                                receive_data_for_write_w = wdata;
                                wready = 1;
                            end
                        end
                        2'b10: begin
                            if (wvalid == 1'b1) begin
                                if ((receive_data_for_write_r >> 4) == 0) begin //address = 0x00
                                    if (wdata[0] == 1'b1) begin //ap_start = 1
                                        state_w = STATE_EXECUTE;
                                    end
                                end
                                else if ((receive_data_for_write_r >> 4) == 1) begin //address = 0x10
                                    data_length_w = wdata;
                                end
                                else begin //address = 0x20, direct communicate with BRAM
                                    tap_EN = 1;
                                    tap_WE = 4'b1111;
                                    tap_A = receive_data_for_write_r-'h20;
                                    tap_Di = wdata;
                                    data_EN = 1;
                                    data_WE = 4'b1111;
                                    data_A = receive_data_for_write_r-'h20;
                                    data_Di = 0; //initialization
                                end
                                wready = 1;
                                write_state_w = 2'b00;
                            end
                        end
                        2'b01: begin
                            if (awvalid == 1'b1) begin
                                if ((awaddr >> 4) == 0) begin //address = 0x00
                                    if (receive_data_for_write_r[0] == 1'b1) begin //ap_start = 1
                                        state_w = STATE_EXECUTE;
                                    end
                                end
                                else if ((awaddr >> 4) == 1) begin //address = 0x10
                                    data_length_w = receive_data_for_write_r;
                                end
                                else begin //address = 0x20, direct communicate with BRAM
                                    tap_EN = 1;
                                    tap_WE = 4'b1111;
                                    tap_A = awaddr-'h20;
                                    tap_Di = receive_data_for_write_r;
                                    data_EN = 1;
                                    data_WE = 4'b1111;
                                    data_A = awaddr-'h20;
                                    data_Di = 0; //initialization
                                end
                                awready = 1;
                                write_state_w = 2'b00;
                            end
                        end
                    endcase
                end
            end
            STATE_EXECUTE: begin
                if ((ss_tvalid == 1'b1) & (pending_sm_tready_r != 1'b1)) begin //second condition prevents fir result from being changed
                    if (bram_wait_r == 1'b1) begin
                        tap_EN = 1;
                        tap_A = counter_r << 2;
                        data_EN = 1;
                        bram_wait_w = 0;

                        if ((($signed({1'b0, counter_r}) + $signed({1'b0, bram_first_data_position_r})) - Tape_Num) >= 0) begin
                            data_A = (($signed({1'b0, counter_r}) + $signed({1'b0, bram_first_data_position_r})) - Tape_Num) << 2;
                        end
                        else begin
                            data_A = ($signed({1'b0, counter_r}) + $signed({1'b0, bram_first_data_position_r})) << 2;
                        end
                        counter_w = counter_r;
                    end
                    else begin
                        tap_EN = 1;
                        data_EN = 1;
                        fir_result_w = fir_result_r + data_Do * tap_Do;
                        if (counter_r == 0) begin
                            fir_result_w = ss_tdata * tap_Do;
                            data_WE = 4'b1111;
                            data_A = bram_first_data_position_r << 2;
                            data_Di = ss_tdata;
                        end
                        if (counter_r == Tape_Num-1) begin
                            pending_sm_tready_w = 1;
                            if (bram_first_data_position_w - 1 != -1)
                                bram_first_data_position_w = bram_first_data_position_r - 1;
                            else
                                bram_first_data_position_w = Tape_Num - 1;
                            if (ss_tlast == 1'b1) begin
                                last_w = 1;
                            end
                            ss_tready = 1;
                        end
                        counter_w = counter_r + 1;
                        bram_wait_w = 1;
                    end
                end
            end
            STATE_CLEAN_DATA_IN_BRAM: begin
                data_EN = 1;
                data_WE = 4'b1111;
                data_A = counter_r << 2;
                data_Di = 0; //initialization
                if (counter_r == Tape_Num-1) begin
                    state_w = STATE_RECEIVE_PARAMETERS;
                end
                else begin
                    counter_w = counter_r + 1;
                end
            end
        endcase
        if (pending_sm_tready_r == 1'b1) begin
            sm_tvalid = 1'b1;
            sm_tdata = fir_result_r;
            if (last_r == 1) begin
                sm_tlast = 1;
            end
            if (sm_tready == 1'b1) begin
                fir_result_w = 0;
                if (last_r == 1'b1) begin
                    state_w = STATE_CLEAN_DATA_IN_BRAM;
                    last_w = 0;
                end
                pending_sm_tready_w = 0;
            end
        end
        if (arvalid == 1'b1) begin
            pending_rready_w = 1;
            receive_address_for_read_w = araddr;
            arready = 1;
            if (((araddr >> 4) != 0) & ((araddr >> 4) != 1)) begin
                tap_EN = 1;
                tap_A = araddr-'h20;
            end
        end
        if (pending_rready_r == 1'b1) begin
            rvalid = 1;
            if ((receive_address_for_read_r >> 4) == 0) begin //address = 0x00
                rdata = {(state_r != STATE_EXECUTE), (state_r != STATE_EXECUTE), 1'b0};
            end
            else if ((receive_address_for_read_r >> 4) == 1) begin //address = 0x10
                rdata = data_length_r;
            end
            else begin //address = 0x20
               rdata = tap_Do;
               tap_EN = 1;
               tap_A = receive_address_for_read_r-'h20;
            end
            if (rready == 1'b1) begin
                pending_rready_w = 0;
            end
        end
    end
    
    always@(posedge axis_clk) begin
        if (axis_rst_n == 1'b0) begin
            state_r <= STATE_RECEIVE_PARAMETERS;
            counter_r <= 0;
            receive_data_for_write_r <= 0;
            write_state_r <= 0;
            data_length_r <= 0;
            fir_result_r <= 0;
            bram_wait_r <= 1;
            bram_first_data_position_r <= 0;
            pending_rready_r <= 0;
            last_r <= 0;
            pending_sm_tready_r <= 0;
            receive_address_for_read_r <= 0;
        end
        else begin
            state_r <= state_w;
            counter_r <= counter_w;
            receive_data_for_write_r <= receive_data_for_write_w;
            write_state_r <= write_state_w;
            data_length_r <= data_length_w;
            fir_result_r <= fir_result_w;
            bram_wait_r <= bram_wait_w;
            bram_first_data_position_r <= bram_first_data_position_w;
            pending_rready_r <= pending_rready_w;
            last_r <= last_w;
            pending_sm_tready_r <= pending_sm_tready_w;
            receive_address_for_read_r <= receive_address_for_read_w;
        end
    end

endmodule