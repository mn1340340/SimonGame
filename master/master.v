module master(
   //global inout
   input CLOCK_50, 
   input [3:0] KEY,
   input [0:0] SW,
   output [9:0] LEDR,
   output [6:0] HEX5, HEX4, HEX1, HEX0,

   // VGA out
   output [7:0] VGA_R, VGA_G, VGA_B,
   output VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK,

   // Audio inout
   input AUD_ADCDAT,
   inout AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, FPGA_I2C_SDAT,
   output AUD_XCK, AUD_DACDAT, FPGA_I2C_SCLK
);

   //game resets on negedge SW[0]=0  
   wire reset;
   assign reset = SW[0];
   
   //cross module signals
   wire [3:0] global_box_select;
   wire [3:0] button_on;
   wire [9:0] screen_select;
   assign button_on=KEY;

   //slow clock
   wire slow_clk;
   display_clock displayclock(CLOCK_50, slow_clk);

   //input timer
   wire [31:0] input_timer;

   ////HEX display 
   wire [5:0] high_score;

    // using operators to store tens and ones digit of score 
   wire [3:0] current_tens, current_ones;
   assign current_tens = current_score / 10;
   assign current_ones = current_score % 10;
  
   char_seg7 hex0_disp(.v3(current_ones[3]), .v2(current_ones[2]), .v1(current_ones[1]), .v0(current_ones[0]), .HEX(HEX0));
   char_seg7 hex1_disp(.v3(current_tens[3]), .v2(current_tens[2]), .v1(current_tens[1]), .v0(current_tens[0]), .HEX(HEX1));

   //high score
   wire [5:0] current_score;
   wire [3:0] high_tens, high_ones;
   assign high_tens = high_score / 10;
   assign high_ones = high_score % 10;
  
   char_seg7 hex4_disp(.v3(high_ones[3]), .v2(high_ones[2]), .v1(high_ones[1]), .v0(high_ones[0]), .HEX(HEX4));
   char_seg7 hex5_disp(.v3(high_tens[3]), .v2(high_tens[2]), .v1(high_tens[1]), .v0(high_tens[0]), .HEX(HEX5));

   //game logic module
   simon_game_logic game_logic(
       .CLOCK_50(CLOCK_50),
       .slow_clk(slow_clk),
       .reset(reset),
       .KEY(KEY),
       .button_on(button_on),
       .global_box_select(global_box_select),
       .screen_select(screen_select),
       .LEDR(LEDR),
       .input_timer(input_timer),
       .high_score(high_score),
       .current_score(current_score)
   );

   //VGA display module
   simon_vga_display vga_display(
       .CLOCK_50(CLOCK_50),
       .slow_clk(slow_clk),
       .reset(reset),
       .input_timer(input_timer),
       .global_box_select(global_box_select),
       .screen_select(screen_select),
       .VGA_R(VGA_R),
       .VGA_G(VGA_G),
       .VGA_B(VGA_B),
       .VGA_HS(VGA_HS),
       .VGA_VS(VGA_VS),
       .VGA_BLANK_N(VGA_BLANK_N),
       .VGA_SYNC_N(VGA_SYNC_N),
       .VGA_CLK(VGA_CLK)
   );

   //audio module
   simon_audio_player audio_player(
       .CLOCK_50(CLOCK_50),
       .reset(reset),
       .global_box_select(global_box_select),
       .AUD_ADCDAT(AUD_ADCDAT),
       .AUD_BCLK(AUD_BCLK),
       .AUD_ADCLRCK(AUD_ADCLRCK),
       .AUD_DACLRCK(AUD_DACLRCK),
       .AUD_XCK(AUD_XCK),
       .AUD_DACDAT(AUD_DACDAT)
   );
   avconf #(.USE_MIC_INPUT(1)) avc(
       .FPGA_I2C_SCLK(FPGA_I2C_SCLK),
       .FPGA_I2C_SDAT(FPGA_I2C_SDAT),
       .CLOCK_50(CLOCK_50),
       .reset(reset)
   );
endmodule


//game logic
module simon_game_logic(
   input CLOCK_50,
   input slow_clk,
   input reset,
   input [3:0] KEY,
   input [3:0] button_on,
   output reg [3:0] global_box_select,
   output reg [9:0] screen_select,
   output reg [9:0] LEDR,
   output reg [31:0] input_timer,
   output reg [4:0] high_score,
   output reg [4:0] current_score
);
   // FSM states & state tracker
    reg [9:0] state_current = state_idle;

   localparam state_idle=10'b0000000001;
   localparam state_random=10'b0000000010;
   localparam state_notrandom=10'b0000000100;
   localparam state_display=10'b0000001000;
   localparam state_notdisplay=10'b0000010000;
   localparam state_input=10'b0000100000;
   localparam state_check=10'b0001000000;
   localparam state_over=10'b0010000000;
   localparam state_newround=10'b0100000000;
   localparam state_YOUWIN=10'b1000000000;

    //button states (4 keys in total)
   localparam b_one=2'b00, b_two=2'b01, b_three=2'b10, b_four=2'b11;

    //led states 
   localparam led1=10'b0000000001;
   localparam led2=10'b0000000010;
   localparam led3=10'b0000000100;
   localparam led4=10'b0000001000;
   localparam led_off=10'b1000000000;
   localparam led_win=10'b0101010101;
   localparam led_correct=10'b0000001111;
   localparam led_lose=10'b1111111111;

    // registers for player and computer sequences, and positions
   reg [4:0] disp_pos;
   reg [63:0] computer_seq = 64'b0; //(32 max number of sequences *2 width of button)
   reg [63:0] player_seq = 64'b0;
   reg [4:0] computer_pos;
   reg [4:0] player_pos;
   reg[4:0] game_over_delay;

    //LFSR instantiation for random number generation
   wire [15:0] lfsr;
   randomiser r0(CLOCK_50, reset, lfsr);
   wire [1:0] new_rnd_val;
   assign new_rnd_val = lfsr[1:0];

    //key edge detection 
   reg [3:0] button_pressed_last;
   wire [3:0] button_edge;
 
   assign button_edge[0] = button_on[0] & ~button_pressed_last[0];
   assign button_edge[1] = button_on[1] & ~button_pressed_last[1];
   assign button_edge[2] = button_on[2] & ~button_pressed_last[2];
   assign button_edge[3] = button_on[3] & ~button_pressed_last[3];
 
   wire any_button_edge;
   assign any_button_edge = button_edge[0] | button_edge[1] | button_edge[2] | button_edge[3];
 
   always @(posedge slow_clk or negedge reset) begin
       if (~reset) begin
           button_pressed_last <= 4'b1111; //keys are active low 
       end else begin
           button_pressed_last <= button_on;
       end
   end


   // Main game FSM
   always @(posedge slow_clk or negedge reset) begin
       if (~reset) begin //resets entirely 
           computer_seq <= 64'b0;
           player_seq <= 64'b0;
           computer_pos <= 5'b0;
           player_pos <= 5'b0;
           disp_pos <= 5'b0;
           high_score <= 0;
           LEDR <= led_off;
           global_box_select <= 4'b0000;
           screen_select <= 10'b0000000001;
           input_timer <= 0;
           current_score<=0;
           state_current <= state_idle;
       end else begin
           case (state_current)
               state_idle: begin
                   game_over_delay <= 5'd0; // Reset delay counter
                   screen_select <= 10'b0000000001;

                   if (any_button_edge) begin //only resets subgame -- ie. high score is not reset
                       computer_pos <= 5'b0;
                       computer_seq <= 64'b0;
                       player_seq <= 64'b0;
                       player_pos <= 5'b0;
                       disp_pos <= 5'b0;
                       LEDR <= led_off;
                       input_timer <= 0;
                       current_score<=0;
                       state_current <= state_random;
                       global_box_select <= 4'b0000;
                       screen_select <= 10'b0000000000;
                   end
               end
             
               state_random: begin //samples rng and adds to sequence at current position
                   computer_seq[computer_pos*2+:2] <= new_rnd_val;
                   computer_pos <= computer_pos + 1'b1;
                   state_current <= state_notrandom;
               end

               state_notrandom: begin
                   LEDR <= led_off;
                   disp_pos <= 5'b0;
                   state_current <= state_display;
               end

               state_display: begin //display the computer generated sequence
                   case (computer_seq[disp_pos*2+:2])
                       b_one: begin
                           LEDR <= led1;
                           global_box_select <= 4'b1000;
                       end
                       b_two: begin
                           LEDR <= led2;
                           global_box_select <= 4'b0100;
                       end
                       b_three: begin
                           LEDR <= led3;
                           global_box_select <= 4'b0010;
                       end
                       b_four: begin
                           LEDR <= led4;
                           global_box_select <= 4'b0001;
                       end
                   endcase
                   disp_pos <= disp_pos + 1'b1;  //keep track of how many have been displayed
                   state_current <= state_notdisplay;
               end

               state_notdisplay: begin 
                   LEDR <= led_off;
                   global_box_select <= 4'b0000;
                   if (disp_pos >= computer_pos) begin
                       input_timer <= 0;
                       state_current <= state_input; // if all has been displayed, move to get user input
                   end else begin
                       state_current <= state_display; //checks if all of the sequence has been displayed
                   end
               end

               state_input: begin
                   case (1'b1) //adds user input to player sequence and keeps position
                       button_edge[0]: begin 
                           player_seq[player_pos*2+:2] <= b_one;
                           player_pos <= player_pos + 1'b1;
                           LEDR <= led1;
                       end
                       button_edge[1]: begin
                           player_seq[player_pos*2+:2] <= b_two;
                           player_pos <= player_pos + 1'b1;
                           LEDR <= led2;
                       end
                       button_edge[2]: begin
                           player_seq[player_pos*2+:2] <= b_three;
                           player_pos <= player_pos + 1'b1;
                           LEDR <= led3;
                       end
                       button_edge[3]: begin
                           player_seq[player_pos*2+:2] <= b_four;
                           player_pos <= player_pos + 1'b1;
                           LEDR <= led4;
                       end
                   endcase


                   //box and audio peripheral signals via physical hold
                   if (~KEY[0]) begin
                       global_box_select <= 4'b1000;
                   end else if (~KEY[1]) begin
                       global_box_select <= 4'b0100;
                   end else if (~KEY[2]) begin
                       global_box_select <= 4'b0010;
                   end else if (~KEY[3]) begin
                       global_box_select <= 4'b0001;
                   end else begin
                       global_box_select <= 4'b0000;
                   end

                   input_timer<=input_timer+1; //increment timer until user reaches number of seq
             
                   if (player_pos >= computer_pos) begin
                       state_current <= state_check;
                       global_box_select <= 4'b0000;
                   end
                 
                   if (input_timer >= 32'd15) begin //counts to 14s(15*2*.4s) before terminating
                       state_current<=state_over;
                       screen_select <= 10'b0010000000; //display game over screen 
                   end
               end


               state_check: begin //check that player sequence is equal to computer sequence 
                   if (computer_pos == 0) begin
                       state_current <= state_random;
                   end else if (computer_pos == 32 && computer_seq == player_seq) begin
                       state_current <= state_YOUWIN; //user reached end
                   end else if (computer_seq == player_seq) begin
                       LEDR <= led_correct;
                       current_score <= computer_pos;
                       if(computer_pos>=high_score) begin //check if user has beat highscore 
                           high_score<=computer_pos;
                       end
                       state_current <= state_newround; // game continues - moving to add next sequence 
                   end else begin
                       state_current <= state_over; 
                       screen_select <= 10'b0010000000;
                   end
               end

               state_YOUWIN: begin
                   state_current <= state_idle;
                   LEDR <= led_win;
                   global_box_select <= 4'b0000;
                   screen_select <= 10'b0000000001;
               end
             
               state_over: begin
                   screen_select <= 10'b0010000000;
                   state_current <= state_idle;
                   LEDR <= led_lose;
                   global_box_select <= 4'b0000;
              
               // need delay because state switches almost instantaneously
                   if (game_over_delay >= 5'd40) begin // Show for ~6 seconds (20 * 0.3s)
                       game_over_delay <= 5'd0;
                       state_current <= state_idle;
                   end else begin
                       game_over_delay <= game_over_delay + 1;
                   end
               end

               state_newround: begin
                   LEDR <= led_off;
                   player_seq <= 64'b0;
                   player_pos <= 5'b0;
                   input_timer <= 0;
                   state_current <= state_random;
                   global_box_select <= 4'b0000;
               end


               default: begin
                   state_current <= state_idle;
                   global_box_select <= 4'b0000;
                   screen_select <= 10'b0000000001;
               end
           endcase
       end
   end
endmodule


//GRAPHICS
module simon_vga_display(
   input CLOCK_50,
   input slow_clk,
   input reset,
   input [31:0] input_timer,
   input [3:0] global_box_select,
   input [9:0] screen_select,
   output [7:0] VGA_R, VGA_G, VGA_B,
   output VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK
);

   parameter nX = 8;
   parameter nY = 8;
   parameter BOX_SIZE_X = 10'd50;
   parameter BOX_SIZE_Y = 9'd50;

   // screen drawing state
   reg draw_full_screen;
   reg [7:0] screen_x_counter;
   reg [6:0] screen_y_counter;
   reg screen_draw_done; // T or F
   reg [9:0] last_state;

    // Detect state changes
    always @(posedge CLOCK_50) begin
        if (~reset) begin
            last_state <= 10'b0000000001;
            draw_full_screen <= 1'b1;  // Draw full screen on reset
            screen_draw_done <= 1'b0;
        end else begin
            last_state <= screen_select;
        
            // Trigger full screen draw when state changes
            if (last_state != screen_select) begin
            draw_full_screen <= 1'b1;
            screen_draw_done <= 1'b0;
            screen_x_counter <= 8'd0;
            screen_y_counter <= 7'd0;
            end
        
            // Full screen drawing logic
            if (draw_full_screen && !screen_draw_done) begin
                if (screen_x_counter == 8'd159) begin
                    screen_x_counter <= 8'd0;
                    if (screen_y_counter == 7'd119) begin
                    screen_draw_done <= 1'b1;
                    draw_full_screen <= 1'b0;
                    end else begin
                    screen_y_counter <= screen_y_counter + 1'b1;
                    end
                end else begin
                    screen_x_counter <= screen_x_counter + 1'b1;
                end
            end
        end
    end

   wire [14:0] pixel_addr;
   wire [5:0] idle_pixel;
   wire [5:0] over_pixel;

   // Select pixel address based on mode
   wire [14:0] screen_pixel_addr;
   wire [14:0] box_pixel_addr;
   assign screen_pixel_addr = screen_y_counter * 160 + screen_x_counter;

   background_rom new_game_background_inst (
   .clk(CLOCK_50),
   .addr(draw_full_screen ? screen_pixel_addr : box_pixel_addr),
   .data(idle_pixel)
   );
   defparam new_game_background_inst.INIT_FILE = "new_game_background_inst.mif";
   background_rom game_over_background_inst (
   .clk(CLOCK_50),
   .addr(draw_full_screen ? screen_pixel_addr : box_pixel_addr),
   .data(over_pixel)
   );
   defparam game_over_background_inst.INIT_FILE = "game_over_background_inst.mif";

   reg [5:0] background_pixel;

   // Select background based on state
   always @(*) begin
       if (screen_select == 10'b0000000001)
           background_pixel = idle_pixel;   // start screen in state_idle
       else if (screen_select == 10'b0010000000)
           background_pixel = over_pixel; // end screen in state_over
       else
           background_pixel = 6'd0;  // Black during gameplay
   end

   parameter X1 = 50, Y1 = 30, COLOR1 = 6'b100000, COLOR1b = 6'b110101;//red, top left, key0
   parameter X2 = 110, Y2 = 30, COLOR2 = 6'b001000, COLOR2b = 6'b011101;//green, top right, key1
   parameter X3 = 50, Y3 = 90, COLOR3 = 6'b000010, COLOR3b = 6'b010111;//blue, bottom left, key 3
   parameter X4 = 110, Y4 = 90, COLOR4 = 6'b101000, COLOR4b = 6'b111101;//yellow, bottom right, key 2


   // FSM combinational logic
   localparam A=2'b00, B=2'b01, C=2'b10, D=2'b11;


   //timer indicator
   parameter X5 = 150, Y5 = 10;        
   parameter timer_size_x = 10'd9;    
   parameter timer_size_y = 9'd9;


   reg [5:0] timer_color;
   always @(posedge slow_clk or negedge reset) begin
       if(~reset)
           timer_color<=6'b111111;
       else if (input_timer < 3)
           timer_color <= 6'b001000;  //green
       else if (input_timer < 7)
           timer_color <= 6'b111101;   //yellow
       else if (input_timer < 12)
           timer_color <= 6'b111000;  //orange
       else
           timer_color <= 6'b100000;  //red
   end

   reg [2:0] box_select = 0;
   reg [1:0] y_Q, Y_D;
   reg [5:0] color;
   reg [nX-1:0] X;
   reg [nY-1:0] Y;
   reg [nX-1:0] current_size_x;
   reg [nY-1:0] current_size_y;

   // Clock domain crossing synchronizer
   reg [3:0] global_box_select_syncing;
   reg [3:0] global_box_select_synced;


    always @(posedge CLOCK_50) begin
        if (~reset || draw_full_screen || box_select >= 5) begin
            box_select <= 0;
        end else if (y_Q == D) begin
            box_select <= box_select + 1;
        end
    end

   //make indicator only flash every other slow clk cycle
   reg flash;
   always @(posedge slow_clk or negedge reset) begin
       if (~reset)
           flash<= 1'b0;
       else
           flash<= ~flash;
   end


   always @(posedge CLOCK_50 or negedge reset) begin
       if (~reset) begin
           global_box_select_syncing <= 4'b0000;
           global_box_select_synced <= 4'b0000;
           X <= X1;
           Y <= Y1;
           color <= COLOR1;
       end else begin
           global_box_select_syncing <= global_box_select;
           global_box_select_synced <= global_box_select_syncing;


           case (box_select)
               3'd0: begin
                   X <= X1; Y <= Y1;
                   color <= (global_box_select_synced == 4'b0001) ? COLOR1b : COLOR1;
                   current_size_x <= BOX_SIZE_X;
                   current_size_y <= BOX_SIZE_Y;
               end
               3'd1: begin
                   X <= X2; Y <= Y2;
                   color <= (global_box_select_synced == 4'b0010) ? COLOR2b : COLOR2;
                   current_size_x <= BOX_SIZE_X;
                   current_size_y <= BOX_SIZE_Y;
               end
               3'd2: begin
                   X <= X3; Y <= Y3;
                   color = (global_box_select_synced == 4'b0100) ? COLOR3b : COLOR3;
                   current_size_x <= BOX_SIZE_X;
                   current_size_y <= BOX_SIZE_Y;
               end
               3'd3: begin
                   X <= X4; Y <= Y4;
                   color = (global_box_select_synced == 4'b1000) ? COLOR4b : COLOR4;
                   current_size_x <= BOX_SIZE_X;
                   current_size_y <= BOX_SIZE_Y;
               end
               3'd4: begin
                   X <= X5; Y <= Y5;
                   color <= flash? timer_color: 6'b000000;  // Flash on/off
                   current_size_x <= timer_size_x;
                   current_size_y <= timer_size_y;
               end
       endcase
       end
   end

   wire go = 1'b1;
   wire [nX-1:0] size_x = current_size_x;
   wire [nY-1:0] size_y = current_size_y;
   wire [nX-1:0] XC;
   wire [nY-1:0] YC;
   wire [nX-1:0] current_x;
   wire [nY-1:0] current_y;

   assign current_x = X - (size_x >> 1) + XC;
   assign current_y = Y - (size_y >> 1) + YC;
   assign box_pixel_addr = current_y * 160 + current_x;

   reg write, Lxc, Lyc, Exc, Eyc;

   // Output to VGA - choose between full screen draw or box draw
   wire [7:0] vga_x_out;
   wire [6:0] vga_y_out;
   wire [5:0] vga_color_out;
   wire vga_write_out;

   assign vga_x_out = draw_full_screen ? screen_x_counter : current_x;
   assign vga_y_out = draw_full_screen ? screen_y_counter : current_y;
   assign vga_color_out = draw_full_screen ? background_pixel : (write ? color : background_pixel);
   assign vga_write_out = draw_full_screen ? 1'b1 : write;


   Up_count U1 ({nX{1'd0}}, CLOCK_50, reset, Lxc, Exc, XC);
       defparam U1.n = nX;
   Up_count U2 ({nY{1'd0}}, CLOCK_50, reset, Lyc, Eyc, YC);
       defparam U2.n = nY;

   always @ (*)
       case (y_Q)
           A:  Y_D = go ? B : A;
           B:  Y_D = (XC != size_x-1) ? B : C;
           C:  Y_D = (YC != size_y-1) ? B : D;
           D:  Y_D = A;
       endcase
     
   always @ (*) begin
       write = 1'b0; Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0;  
       // Only draw boxes if NOT drawing full screen AND NOT in idle/game over states
       if (!draw_full_screen && screen_select != 10'b0000000001 && screen_select != 10'b0010000000)
        begin // Not game over
        case (y_Q)
            A:  begin Lxc = 1'b1; Lyc = 1'b1; end
            B:  begin Exc = 1'b1; write = 1'b1; end
            C:  begin Lxc = 1'b1; Eyc = 1'b1; end
            D:  Lyc = 1'b1;
        endcase
        end
    end

   always @(posedge CLOCK_50 or negedge reset) begin
       if (~reset) begin
           y_Q <= A;
       end else if (draw_full_screen) begin
         y_Q <= A;
       end else begin
           y_Q <= Y_D;
       end
   end
  
      vga_adapter vga(
       .resetn(reset),
       .clock(CLOCK_50),
       .color(vga_color_out),  // not sure if this will mess smth up
       .x(vga_x_out),
       .y(vga_y_out),
       .write(vga_write_out),  // also not sure if this will work
       .VGA_R(VGA_R),
       .VGA_G(VGA_G),
       .VGA_B(VGA_B),
       .VGA_HS(VGA_HS),
       .VGA_VS(VGA_VS),
       .VGA_BLANK_N(VGA_BLANK_N),
       .VGA_SYNC_N(VGA_SYNC_N),
       .VGA_CLK(VGA_CLK)
   );
endmodule


//AUDIO
module simon_audio_player(
   input CLOCK_50,
   input reset,
   input [4:0] global_box_select,
   input AUD_ADCDAT,
   inout AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK,
   output AUD_XCK, AUD_DACDAT
);

   wire audio_in_available;
   reg [31:0] left_channel_audio_in;
   reg [31:0] right_channel_audio_in;
   wire read_audio_in;

   wire audio_out_allowed;
   wire [31:0] left_channel_audio_out;
   wire [31:0] right_channel_audio_out;
   reg write_audio_out;

   wire [31:0] q_data;
   wire [31:0] q_data_B, q_data_G, q_data_E, q_data_C;

   reg playback;
   reg [12:0] addr_cnt;
   reg is_lower;
   reg delay;
   reg enable;
   reg [1:0] selected_audio;

   // Mux for selecting audio source
   assign q_data = (selected_audio == 2'b00) ? q_data_B :
                   (selected_audio == 2'b01) ? q_data_G :
                   (selected_audio == 2'b10) ? q_data_E :
                   q_data_C;

   // Clock domain crossing synchronizer
   reg [3:0] global_box_select_syncing;
   reg [3:0] global_box_select_synced;
   reg [3:0] global_box_select_last;

   always @(posedge CLOCK_50 or negedge reset) begin
       if (~reset) begin
           global_box_select_syncing <= 4'b0000;
           global_box_select_synced <= 4'b0000;
         
       end else begin
           global_box_select_syncing <= global_box_select;
           global_box_select_synced <= global_box_select_syncing;
           global_box_select_last<=global_box_select_synced;
       end
   end
   //edge detection
   wire audio_edge;
   assign audio_edge = (global_box_select_synced != 4'b0000) && (global_box_select_last == 4'b0000);

   // Audio playback control- same as vga signal
   always @(posedge CLOCK_50 or negedge reset) begin
       if (~reset) begin
           addr_cnt <= 13'b0;
           write_audio_out <= 1'b0;
           playback <= 1'b0;
           selected_audio <= 2'b00;

       end else if (audio_edge) begin 
           playback <= 1'b1;
           addr_cnt <= 13'b0;
           write_audio_out <= 1'b0;
         
           case (global_box_select_synced)
               4'b0001: selected_audio <= 2'b00;
               4'b0010: selected_audio <= 2'b01;
               4'b0100: selected_audio <= 2'b10;
               4'b1000: selected_audio <= 2'b11;
           endcase

       end else if (audio_out_allowed && playback) begin
           if (enable)
               addr_cnt <= addr_cnt + 1;
           write_audio_out <= 1'b1;
           if (addr_cnt == 13'd8191 && enable)
               playback <= 1'b0;
       end
   end

   // Down sampling logic
   always @(posedge CLOCK_50 or negedge reset) begin
       if (~reset) begin
           enable <= 1'b0;
           is_lower <= 1'b0;
           delay <= 1'b0;
       end else if (audio_out_allowed && playback) begin
           if (delay && is_lower) begin
               enable <= 1'b1;
               delay <= 1'b0;
           end else if (delay) begin
               enable <= 1'b0;
           end else if (is_lower) begin
               delay <= ~delay;
               enable <= 1'b0;
           end else begin
               enable <= 1'b0;
           end
           is_lower <= ~is_lower;
       end
   end

   assign left_channel_audio_out = (is_lower) ? {q_data[15:0], 16'b0} : {q_data[31:16], 16'b0};
   assign right_channel_audio_out = (is_lower) ? {q_data[15:0], 16'b0} : {q_data[31:16], 16'b0};
 
   //rom inst
   piano_key_B piano_key_B_inst (
       .address ( addr_cnt ),
       .clock ( CLOCK_50 ),
       .data (  ),
       .q ( q_data_B )
   );

   piano_key_G piano_key_G_inst(
       .address ( addr_cnt ),
       .clock ( CLOCK_50 ),
       .data (  ),
       .q ( q_data_G )
   );

   piano_key_E piano_key_E_inst (
       .address ( addr_cnt ),
       .clock ( CLOCK_50 ),
       .data (  ),
       .q ( q_data_E )
   );

   piano_key_C piano_key_C_inst(
       .address ( addr_cnt ),
       .clock ( CLOCK_50 ),
       .data (  ),
       .q ( q_data_C )
   );

   // Audio controller
   Audio_Controller Audio_Controller (
       .CLOCK_50(CLOCK_50),
       .reset(~reset),
       .clear_audio_in_memory(),
       .read_audio_in(read_audio_in),
       .clear_audio_out_memory(),
       .left_channel_audio_out(left_channel_audio_out),
       .right_channel_audio_out(right_channel_audio_out),
       .write_audio_out(write_audio_out),
       .AUD_ADCDAT(AUD_ADCDAT),
       .AUD_BCLK(AUD_BCLK),
       .AUD_ADCLRCK(AUD_ADCLRCK),
       .AUD_DACLRCK(AUD_DACLRCK),
       .audio_in_available(audio_in_available),
       .left_channel_audio_in(),
       .right_channel_audio_in(),
       .audio_out_allowed(audio_out_allowed),
       .AUD_XCK(AUD_XCK),
       .AUD_DACDAT(AUD_DACDAT)
   );
endmodule

////////////////////////HELPERS
module randomiser(clk, reset, lfsr);
   input clk, reset;
   output reg [15:0] lfsr;
   wire feedback;
   assign feedback = lfsr[15] ^ lfsr[14] ^ lfsr[3] ^ lfsr[0];//taps from maximal lfsr wiki page

   always @(posedge clk or negedge reset) begin
       if (~reset) begin
           lfsr <= 16'hECE2;
       end else begin
           lfsr <= {lfsr[14:0], feedback};
       end
   end
endmodule

module display_clock(clk, slow_clk);
   input clk;
   output reg slow_clk;
   reg [27:0] counter = 0;
 
   initial begin
       slow_clk = 0; 
   end

   always @(posedge clk) begin
       if (counter >= 28'd14999999) begin
           counter <= 0;
           slow_clk <= ~slow_clk;
       end else begin
           counter <= counter + 1;
       end
   end
endmodule


//for vga display
module Up_count (R, clk, resetn, L, E, Q);
   parameter n = 8;
   input [n-1:0] R;
   input clk, resetn, E, L;
   output reg [n-1:0] Q;

   always @ (posedge clk)
       if (~resetn)
           Q <= {n{1'b0}};
       else if (L == 1)
           Q <= R;
       else if (E)
           Q <= Q + 1'b1;
endmodule

//for hex display
module char_seg7(v3, v2, v1, v0, HEX);
   input v3, v2, v1, v0;
   output [6:0] HEX;


   assign HEX[0]=(v2&~v1&~v0)|(~v3&~v2&~v1&v0);
   assign HEX[1]=(v2&v1&~v0)|(v2&~v1&v0);
   assign HEX[2]=(~v2&v1&~v0);
   assign HEX[3]=(~v3&~v2&~v1&v0)|(v2&~v1&~v0)|(v2&v1&v0);
   assign HEX[4]=(v0|(v2&~v1&~v0));
   assign HEX[5]=(v1&v0)|(~v3&~v2&~v1&v0)|(~v2&v1&~v0);
   assign HEX[6]=(~v3&~v2&~v1)|(v2&v1&v0);
endmodule

module background_rom(clk, addr, data);
  parameter INIT_FILE = "background.mif";
  parameter ADDR_WIDTH = 15;  // Adjust based on resolution
  parameter DATA_WIDTH = 6;  // 6-bit RGB

  input clk;
  input [ADDR_WIDTH-1:0] addr;
  output reg [DATA_WIDTH-1:0] data;

  (* ram_init_file = INIT_FILE *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
 
  always @(posedge clk) begin
      data <= mem[addr];
  end
endmodule
