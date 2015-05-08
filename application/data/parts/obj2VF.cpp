#!/bin/bash
/*/../bin/ls > /dev/null
# BEGIN BASH SCRIPT 
export PS4=""
set -o xtrace       
printf "//" | cat - $0 | g++ -std=c++11  -o .main -x c++ - -I/opt/local/include/eigen3/ -I/usr/local/igl/igl_lib/include -L/usr/local/igl/igl_lib/lib -ligl && ./.main $@
rm -f .main         
# END BASH SCRIPT
exit
*/

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/writeDMAT.h>
#include <igl/readDMAT.h>
#include <igl/pathinfo.h>
#include <iostream>
#include <string>

// ./obj2VF.cpp *.obj
int main(int argc,char * argv[])
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  MatrixXd V;
  MatrixXi F;
  for(int f = 1;f<argc;f++)
  {
    readOBJ(argv[f],V,F);
    string dirname, basename, extension, filename;
    pathinfo(argv[f],dirname,basename,extension,filename);
    writeOBJ(dirname+"/"+filename+".copy.obj",V,F);
    writeDMAT(dirname+"/"+filename+".V.dmat",V,false);
    writeDMAT(dirname+"/"+filename+".F.dmat",F,false);
    readDMAT(dirname+"/"+filename+".V.dmat",V);
    readDMAT(dirname+"/"+filename+".F.dmat",F);
    writeOBJ(dirname+"/"+filename+".dmat.obj",V,F);
  }
}
