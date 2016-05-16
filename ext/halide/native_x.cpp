
#include <Halide.h>

#include <cstdio>
using namespace Halide;

#pragma comment(lib, "Halide.lib") 

void convertYCbCr()
{
    ImageParam input(UInt(8), 3, "input_convertcbcr");
    Func ycbcr("convertYCbCr");
    Var x("x"), y("y"), yi("yi"), c("c");

    Func gray("lightness");
    Func red("red");
    Func blue("blue");

    gray(x, y) = input(0, x, y) * 0.299f + input(1, x, y) * 0.587f + 0.114f * input(2, x, y);
    red(x, y) = input(0, x, y);
    blue(x, y) = input(2, x, y);


    // The algorithm
    ycbcr(c, x, y) = cast<uint8_t>(select(c == 0, gray(x, y),
                            select(c == 2, ((red(x, y) - gray(x, y))*0.564f + 128),
                                     ((blue(x, y) - gray(x, y))*0.713f + 128)
                                     )
                            ));
    //ycbcr(c, x, y) = input(c, x, y);

    //ycbcr.trace_stores();

    // How to schedule it
    ycbcr.split(y, y, yi, 8).parallel(y).vectorize(x, 8);
    ycbcr.parallel(y);
    Halide::Target t = Halide::get_target_from_environment();
    if(t.arch == Halide::Target::ARM)
    {
        std::vector<Argument> args;
        args.push_back(input);
        ycbcr.compile_to_assembly("convertYCbCr_x.s", args, "convertYCbCr_x");
        ycbcr.compile_to_header("convertYCbCr_x.h", args, "convertYCbCr_x");
    } else {
		/*
		if (t.os == Halide::Target::Windows && t.bits == 32)
		{
			std::vector<Argument> args;
			args.push_back(input);
			ycbcr.compile_to_assembly("convertYCbCr_x32.s", args, "convertYCbCr_x");
		}
		else */{
			std::vector<Argument> args;
			args.push_back(input);
			ycbcr.compile_to_file("convertYCbCr_x", args);
		}
    }
}

void getChromeMaskNV21()
{
    ImageParam input(UInt(8), 2, "input_nv21");
    Func mask("mask");
    Func map("map");

    const int16_t BLUENESS_THRESH = 105;
    const int16_t REDNESS_THRESH = 120;
    const int16_t MASK_THRESH = 100;


    Var x("x"), y("y"), xsub("xsub"), ysub("ysub"), yi("yi"), xi("xi");

    Func mapsub("mapsub");
    Func masksub("masksub");
    Func invCb("invCb");
    Func invCr("invCr");
    Func ptuple("ptuple");

    Expr offset = input.height()*2/3;

    invCb(xsub, ysub) = 255 - cast<int16_t>(input(2*xsub, offset + ysub));
    invCr(xsub, ysub) = 255 - cast<int16_t>(input(2*xsub + 1, offset + ysub));
    //masksub(xsub, ysub) = ( cast<uint16_t>((MASK_THRESH - ((invCb(xsub, ysub) - BLUENESS_THRESH)*(invCr(xsub, ysub) - REDNESS_THRESH)))) >> 15 )*255;
    masksub(xsub, ysub) = cast<int16_t>( (invCb(xsub, ysub) - BLUENESS_THRESH)*(invCr(xsub,ysub) - REDNESS_THRESH) > MASK_THRESH  )*255;

    mapsub(xsub, ysub) = clamp(2*(invCr(xsub, ysub) - invCb(xsub, ysub)) + 128, 0, 255);

    ptuple(x, y) = Tuple(cast<uint8_t>(mapsub(x/2, y/2)), cast<uint8_t>(masksub(x/2, y/2)));

    Halide::Target t = Halide::get_target_from_environment();
    if(t.arch == Halide::Target::ARM)
    {
        invCb.store_at(ptuple, yi).compute_at(ptuple, x);//.vectorize(xsub, 2);
        invCr.store_at(ptuple, yi).compute_at(ptuple, x);//.vectorize(xsub, 2);
        ptuple.split(y, y, yi, 4).split(x, x, xi, 4).parallel(y).vectorize(x, 4);
        std::vector<Argument> args;
        args.push_back(input);
        ptuple.compile_to_header("getChromeMaskNV21_tuple.h", args, "getChromeMaskNV21_tuple");
        ptuple.compile_to_assembly("getChromeMaskNV21_tuple.s", args, "getChromeMaskNV21_tuple");
    } else {
		/*
		if (t.os == Halide::Target::Windows && t.bits == 32)
		{
			std::vector<Argument> args;
			args.push_back(input);
			ptuple.compile_to_assembly("getChromeMaskNV21_tuple32.s", args, "getChromeMaskNV21_tuple");
		}
		else */{
			std::vector<Argument> args;
			args.push_back(input);
			ptuple.compile_to_file("getChromeMaskNV21_tuple", args);
		}
    }
}


void getChromeMaskYCbCr()
{
    ImageParam input(UInt(8), 3, "input_cbcr");
    Func mask("mask");
    Func map("map");

    const int16_t BLUENESS_THRESH = 105;
    const int16_t REDNESS_THRESH = 120;
    const int16_t MASK_THRESH = 100;


    Var x("x"), y("y"), yi("yi");

    Func invCb("invCb");
    Func invCr("invCr");

    invCb(x, y) = 255 - cast<int16_t>(input(1, x, y));
    invCr(x, y) = 255 - cast<int16_t>(input(2, x, y));
    mask(x, y) = cast<uint8_t>(select(((invCb(x, y) - BLUENESS_THRESH)*(invCr(x, y) - REDNESS_THRESH))>MASK_THRESH, 255, 0));
    map(x, y) = cast<uint8_t>(clamp(2*(invCb(x, y) - invCr(x, y)) + 128, 0, 255));


    mask.split(y, y, yi, 8).parallel(y).vectorize(x, 8);
    mask.parallel(y);

    map.split(y, y, yi, 8).parallel(y).vectorize(x, 8);
    map.parallel(y);

    Halide::Target t = Halide::get_target_from_environment();
    if(t.arch == Halide::Target::ARM)
    {
        std::vector<Argument> args;
        args.push_back(input);
        map.compile_to_assembly("getChromeMaskYCbCr_map_x.s", args, "getChromeMaskYCbCr_map_x");
        mask.compile_to_assembly("getChromeMaskYCbCr_mask_x.s", args, "getChromeMaskYCbCr_mask_x");
        map.compile_to_header("getChromeMaskYCbCr_map_x.h", args, "getChromeMaskYCbCr_map_x");
        mask.compile_to_header("getChromeMaskYCbCr_mask_x.h", args, "getChromeMaskYCbCr_mask_x");
    } else {
		/*
		if (t.os == Halide::Target::Windows && t.bits == 32)
		{
			std::vector<Argument> args;
			args.push_back(input);
			map.compile_to_assembly("getChromeMaskYCbCr_map_x32.s", args, "getChromeMaskYCbCr_map_x");
			mask.compile_to_assembly("getChromeMaskYCbCr_mask_x32.s", args, "getChromeMaskYCbCr_mask_x");
		}
		else */{
			std::vector<Argument> args;
			args.push_back(input);
			map.compile_to_file("getChromeMaskYCbCr_map_x", args);
			mask.compile_to_file("getChromeMaskYCbCr_mask_x", args);
		}
    }
}


void smooth3x3_ui8()
{
    ImageParam input(UInt(8), 2, "input_smooth");
    Func input_16("input_16");
    Func blur_x("blur_x"), blur_y("blur_y");
    Var x("x"), y("y"), xi("xi"), yi("yi");


    Expr clamped_x = clamp(x, 0, input.width()-1);
    // Similarly clamp y.
    Expr clamped_y = clamp(y, 0, input.height()-1);

    // The algorithm
    input_16(x, y) = cast<uint16_t>(input(clamped_x, clamped_y));

    blur_x(x, y) = (input_16(x, y)*2 + input_16(x-1, y) + input_16(x+1, y)) / 4;
    blur_y(x, y) = cast<uint8_t>((blur_x(x, y)*2 + blur_x(x, y-1) + blur_x(x, y+1)) /4);

    //TODO make the sheduling different for different architectures

    Halide::Target t = Halide::get_target_from_environment();
    if(t.arch == Halide::Target::ARM)
    {
        std::vector<Argument> args;
        args.push_back(input);
        blur_y.compile_to_assembly("smooth3x3_ui8.s", args, "smooth3x3_ui8");
        blur_y.compile_to_header("smooth3x3_ui8.h", args, "smooth3x3_ui8");
    } else {
        // How to schedule it
        blur_y.split(y, y, yi, 8).parallel(y).vectorize(x, 8);
        blur_x.store_at(blur_y, y).compute_at(blur_y, yi).vectorize(x, 8);

		/*
		if (t.os == Halide::Target::Windows && t.bits == 32)
		{
			std::vector<Argument> args;
			args.push_back(input);
			blur_y.compile_to_assembly("smooth3x3_ui832.s", args, "smooth3x3_ui8");
		}
		else */{

			std::vector<Argument> args;
			args.push_back(input);
			blur_y.compile_to_file("smooth3x3_ui8", args);
		}
    }
}

void computeGradients3x3()
{
    ImageParam input(UInt(8), 2, "input_grad");
    Func input_f("input_f");
    Func grad_x("grad_x"), grad_y("grad_y");
    Var x("x"), y("y");

    input_f(x, y) = cast<float>(input(x, y))*0.25f;
    grad_x(x, y) = (input_f(x+1, y) - input_f(x - 1, y));
    grad_y(x, y) = (input_f(y, x+1) - input_f(y, x - 1));

    grad_x.parallel(y).vectorize(x, 4);
	//grad_y.parallel(y).vectorize(x, 4);
    //grad_y.reorder(y, x).parallel(x).vectorize(y, 4);

    Halide::Target t = Halide::get_target_from_environment();
    if(t.arch == Halide::Target::ARM)
    {
        std::vector<Argument> args;
        args.push_back(input);

        grad_x.compile_to_assembly("computeGradients3x3_X.s", args, "computeGradients3x3_X");
        grad_y.compile_to_assembly("computeGradients3x3_Y.s", args, "computeGradients3x3_Y");
        grad_x.compile_to_header("computeGradients3x3_X.h", args, "computeGradients3x3_X");
        grad_y.compile_to_header("computeGradients3x3_Y.h", args, "computeGradients3x3_Y");
    } else {
		/*
		if (t.os == Halide::Target::Windows && t.bits == 32)
		{
			std::vector<Argument> args;
			args.push_back(input);
			grad_x.compile_to_assembly("computeGradients3x3_X32.s", args, "computeGradients3x3_X");
			grad_y.compile_to_assembly("computeGradients3x3_Y32.s", args, "computeGradients3x3_Y");
		}
		else */{
			std::vector<Argument> args;
			args.push_back(input);
			grad_x.compile_to_file("computeGradients3x3_X", args);
			grad_y.compile_to_file("computeGradients3x3_Y", args);
		}
    }
}

void subsample2()
{
    ImageParam input(UInt(8), 2, "input_subsample");
    Func input_16("input_16");
    Func output("output");
    Var x, y;

    // The algorithm
    input_16(x, y) = cast<uint16_t>(input(x, y));

    output(x, y) = cast<uint8_t>((input_16(2*x, 2*y) + input_16(2*x+1, 2*y) + input_16(2*x, 2*y+1) + input_16(2*x+1, 2*y + 1))/4);

    // How to schedule it
    std::vector<Argument> args;
    args.push_back(input);

    Halide::Target t = Halide::get_target_from_environment();
    if(t.arch == Halide::Target::ARM)
    {
        output.compile_to_assembly("halide_subsample2.s", args, "halide_subsample2");
        output.compile_to_header("halide_subsample2.h", args, "halide_subsample2");
    } else {

		output.vectorize(x, 4);
		/*
		if (t.os == Halide::Target::Windows && t.bits == 32)
		{
			std::vector<Argument> args;
			args.push_back(input);
			output.compile_to_assembly("halide_subsample232.s", args, "halide_subsample2");
		}
		else */{
			std::vector<Argument> args;
			args.push_back(input);
			output.compile_to_file("halide_subsample2", args);
		}
    }
}

void subsample4()
{
    ImageParam input(UInt(8), 2, "input_subsample4");
    Func input_16("input_16");
    Func output("output");
    Var x, y;

    // The algorithm
    input_16(x, y) = cast<uint16_t>(input(x, y));

    output(x, y) = cast<uint8_t>((
                (input_16(4*x, 4*y) + input_16(4*x+1, 4*y) + input_16(4*x + 2, 4*y) + input_16(4*x+3, 4*y )) +
                (input_16(4*x, 4*y + 1) + input_16(4*x+1, 4*y + 1) + input_16(4*x + 2, 4*y + 1) + input_16(4*x+3, 4*y + 1)) +
                (input_16(4*x, 4*y + 2) + input_16(4*x+1, 4*y + 2) + input_16(4*x + 2, 4*y + 2) + input_16(4*x+3, 4*y + 2)) +
                (input_16(4*x, 4*y + 3) + input_16(4*x+1, 4*y + 3) + input_16(4*x + 2, 4*y + 3) + input_16(4*x+3, 4*y + 3))) /16 // 1/16
                );

    output.vectorize(x, 2);
    // How to schedule it
    std::vector<Argument> args;
    args.push_back(input);
    Halide::Target t = Halide::get_target_from_environment();
    if(t.arch == Halide::Target::ARM)
    {
        output.compile_to_assembly("halide_subsample4.s", args, "halide_subsample4");
        output.compile_to_header("halide_subsample4.h", args, "halide_subsample4");
    } else {
		/*
		if (t.os == Halide::Target::Windows && t.bits == 32)
		{
			std::vector<Argument> args;
			args.push_back(input);
			output.compile_to_assembly("halide_subsample432.s", args, "halide_subsample4");
		}
		else */{
			std::vector<Argument> args;
			args.push_back(input);

			output.compile_to_file("halide_subsample4", args);
		}
    }
}

/*
void compute2by2GradientMatrix()
{
    ImageParam gx(Float(32), 2);
    ImageParam gy(Float(32), 2);
     Func gradientMatrix("gradientMatrix");
     Var x("x"), y("y");
     RDom r(gx); // Iterate over all pixels in the input

     gradientMatrix(x) = 0;
     gradientMatrix(x) = gradientMatrix(x) + gx(r.x, r.y)*gx(r.x, r.y);

     gradientMatrix.compile_to_file("test", gx, gy);
}
*/

int main(int argc, char **argv) {


    convertYCbCr();
    getChromeMaskNV21();
    getChromeMaskYCbCr();
    smooth3x3_ui8();
    computeGradients3x3();
    subsample2();
    subsample4();

    return 0;
}
