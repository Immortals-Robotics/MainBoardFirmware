////////////////////////////////////////////////////////////////////////////////
// SubModule PID
// Created   5/9/2010 8:34:48 PM
////////////////////////////////////////////////////////////////////////////////

/*module abs_a_b ( a , b , o );
input[7:0] a;
input[7:0] b;
output[7:0] o;
assign o = (a==b)?0:(( a > b )? ( a - b ):( b - a ));
endmodule*/

//`include  "Verilog2.V"

module PIDd (Current, Kp, Desired, Dir_C , Dir_D , Speed , Dir_O );

input  [7:0] Current;
input  [7:0] Kp;
input  [7:0] Desired;
input Dir_D;
input Dir_C;
output [7:0] Speed;
output Dir_O;

wire [16:0] pesh = (Dir_C==Dir_D)?((Current>Desired)?(Kp*(Current-Desired)):(Kp*(Desired-Current))):(Kp*(Current+Desired));
wire [10:0] pesh2 = (pesh>255)?255:(pesh);
wire [7:0] tmp_speed = ((Current==Desired)&&(Dir_C==Dir_D))?0:(pesh2[8:1]);
wire tmp_dir_O = (Dir_C==Dir_D)?((Dir_D)?(Current<Desired):(Current>Desired)):(Dir_D);

wire [7:0] tmp_des = Desired / 2;

wire [10:0] speed_f = (tmp_dir_O==Dir_D)?(tmp_speed+tmp_des):((tmp_speed>tmp_des)?(tmp_speed-tmp_des):(tmp_des-tmp_speed));

assign Speed = (speed_f>255)?255:speed_f[7:0];
assign Dir_O = (tmp_dir_O==Dir_D)?(Dir_D):((tmp_speed>tmp_des)?(tmp_dir_O):(Dir_D));




endmodule
////////////////////////////////////////////////////////////////////////////////
