// Lossless image-sampling representation for a 1920x1080 RGB565 source.
// Q10 rays whose four neighbors are all outside the image produce black.
// Canonicalizing only those rays makes signed 22-bit U/V sufficient for
// every observable sample, including partially valid neighbors at -1.
module coordinate_pack_q10(
 input wire signed [31:0] u,v,output wire [43:0] packed_word);
 localparam signed [21:0] BLACK=-22'sd2048;
 wire has_neighbor=u>=-32'sd1024 && u<32'sd1966080 &&
                   v>=-32'sd1024 && v<32'sd1105920;
 assign packed_word=has_neighbor?{u[21:0],v[21:0]}:{BLACK,BLACK};
endmodule

// Reconstruct hints in the existing register stage AFTER coordinate RAM.
// Hints are derived from the stored Q10 values, so they need no RAM bits.
// Keep this combinational codec upstream of the preserved cq*_pipe registers:
// cache RAM addressing still sees the same registered 82-bit interface.
module coordinate_unpack_q10(
 input wire [43:0] packed_word,output wire [81:0] cache_word);
 wire [31:0] u={{10{packed_word[43]}},packed_word[43:22]};
 wire [31:0] v={{10{packed_word[21]}},packed_word[21:0]};
 wire [3:0] even_x=u[17:14]+(&u[13:10]);
 wire [4:0] even_y=v[15:11]+v[10];
 assign cache_word={even_y,v[15:11],even_x,u[17:14],u,v};
endmodule
