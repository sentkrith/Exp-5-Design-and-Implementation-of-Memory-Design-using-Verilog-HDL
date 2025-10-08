# Exp-5-Design-and-Simulate the-Memory-Design-using-Verilog-HDL
#Aim
To design and simulate a RAM,ROM,FIFO using Verilog HDL, and verify its functionality through a testbench in the Vivado 2023.1 environment.
Apparatus Required
Vivado 2023.1
Procedure
1. Launch Vivado 2023.1
Open Vivado and create a new project.
2. Design the Verilog Code
Write the Verilog code for the RAM,ROM,FIFO
3. Create the Testbench
Write a testbench to simulate the memory behavior. The testbench should apply various and monitor the corresponding output.
4. Create the Verilog Files
Create both the design module and the testbench in the Vivado project.
5. Run Simulation
Run the behavioral simulation to verify the output.
6. Observe the Waveforms
Analyze the output waveforms in the simulation window, and verify that the correct read and write operation.
7. Save and Document Results
Capture screenshots of the waveform and save the simulation logs. These will be included in the lab report.

# Code
# RAM
**1.RTL CODE**
```
module mem_1kb(input clk,rst,en,input[7:0]datain,input[9:0]address,output reg[7:0]dataout);
reg [7:0]mem_1kb[1023:0];
always@(posedge clk)
begin
    if(rst)
        dataout<=8'b0;
    else if(en)
        mem_1kb[address]<=datain;
    else
        dataout<=mem_1kb[address];
end
endmodule
```
**2.TESTBENCH CODE**
```
module mem_1kb_tb;
reg clk_t,rst_t,en_t;
reg[7:0]datain_t;
reg[9:0]address_t;
wire[7:0]dataout_t;

mem_1kb dut(.clk(ck_t),.rst(rst_t),.en(en_t),.datain(datain_t),.address(address_t),.dataout(dataout_t));

initial
    begin
        clk_t  = 1'b0;
        rst_t = 1'b1;
    #100
        rst_t = 1'b0;
        en_t = 1'b1;
        address_t = 10'd985;
        datain_t = 8'd55;
     #100
        address_t = 10'd1000;
        datain_t = 8'd125;
     #100
        en_t = 1'b0;
        address_t = 10'd985;
     #100
        address_t = 10'd1000;
     end
     always
     #10 clk_t = ~clk_t;
 
        
endmodule
```
**3.OUTPUT WAVEFORM**
<img width="1919" height="1079" alt="image" src="https://github.com/user-attachments/assets/7cbb06c5-aaf7-4c3d-beb8-4cef12d34908" />


# ROM
**1.RTL CODE**
```
module rom(
    input clk, rst,
    input [9:0] address,
    output reg [7:0] dout
);

    reg [7:0] rom[1023:0];

    initial
    begin
        rom[10'd100] = 8'd42;
        rom[10'd255] = 8'd150;
        rom[10'd520] = 8'd200;
        rom[10'd999] = 8'd255;
    end

    always@(posedge clk)
    begin
        if(rst)
            dout <= 8'b0;
        else
            dout <= rom[address];
    end
endmodule

```
**2.TEST BENCH CODE**
```
module tb_rom;
    reg clk, rst;
    reg [9:0] address;
    wire [7:0] dout;

    rom dut (
        .clk(clk),
        .rst(rst),
        .address(address),
        .dout(dout)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst = 1;
        address = 10'd0;
        #12;
        rst = 0;

        // Test addresses in ROM
        #10 address = 10'd100;
        #10 address = 10'd255;
        #10 address = 10'd520;
        #10 address = 10'd999;
        #10 address = 10'd1023;
        #20;
        $finish;
    end

    
endmodule
```
**3.OUTPUT WAVEFORM**
<img width="1919" height="1079" alt="image" src="https://github.com/user-attachments/assets/7c677176-ed60-40b5-8d34-80c893732637" />

# FIFO
**1.RTL CODE**
```
module syn_fifo #(parameter DEPTH=8, DATA_WIDTH=8)(
   input clk, rst_n,
   input w_en, r_en,
   input [DATA_WIDTH-1:0] data_in,
   output reg [DATA_WIDTH-1:0] data_out,
   output full, empty
);
   reg [$clog2(DEPTH)-1:0] w_ptr, r_ptr;
   reg [DATA_WIDTH-1:0] fifo [0:DEPTH-1];

   always @(posedge clk) begin
       if (!rst_n) begin
           w_ptr <= 0;
           r_ptr <= 0;
           data_out <= 0;
       end
   end

   always @(posedge clk) begin
       if (w_en & !full) begin
           fifo[w_ptr] <= data_in;
           w_ptr <= w_ptr + 1'b1;
       end
   end

   always @(posedge clk) begin
       if (r_en & !empty) begin
           data_out <= fifo[r_ptr];
           r_ptr <= r_ptr + 1'b1;
       end
   end

   assign full  = ((w_ptr + 1'b1) == r_ptr);
   assign empty = (w_ptr == r_ptr);

endmodule
```
 
**2.TESTBENCH CODE**
```
module syn_fifo_tb;

    reg clk, rst_n;
    reg w_en, r_en;
    reg [7:0] data_in;
    wire [7:0] data_out;
    wire full, empty;

    syn_fifo #(8,8) dut (
        .clk(clk),
        .rst_n(rst_n),
        .w_en(w_en),
        .r_en(r_en),
        .data_in(data_in),
        .data_out(data_out),
        .full(full),
        .empty(empty)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    initial begin
        rst_n = 0; w_en = 0; r_en = 0; data_in = 0;
        #25 rst_n = 1;
        @(posedge clk); w_en = 1; data_in = 8'd50;
        @(posedge clk); data_in = 8'd44;
        @(posedge clk); data_in = 8'd45;
        @(posedge clk); w_en = 0;
        @(posedge clk); r_en = 1;
        #50 $finish;
    end

endmodule
```
 
**3.OUTPUT WAVEFORM**
<img width="1899" height="1066" alt="image" src="https://github.com/user-attachments/assets/21e1b070-4886-4d66-94b7-7cf50f2e9bf9" />



# Conclusion
The RAM, ROM, FIFO memory with read and write operations was designed and successfully simulated using Verilog HDL. The testbench verified both the write and read functionalities by simulating the memory operations and observing the output waveforms. The experiment demonstrates how to implement memory operations in Verilog, effectively modeling both the reading and writing processes.
 
 

