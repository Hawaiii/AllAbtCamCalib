
#include "AppleTree.h"


using namespace cv;
void parse_argv(int argc, char** argv) {
  if(argc != 1) {
    printf("Incorrect usage: Demo image_dir\n");
    exit(-1);
  }
}


int main(int argc, char** argv) {

  parse_argv(argc, argv);

  std::cout<<"starts from here...\n";
  AppleJuice* tree = new AppleJuice();
  juice->ReadLookup_table("../../calib/h600.csv");
   
   return 0;
  }
