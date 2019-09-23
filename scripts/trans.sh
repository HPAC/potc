markus@markus-notebook:~/Documents/potc/src$ sed -i 's/__m512d/fvec/g' gen_intel_visitor.h 
markus@markus-notebook:~/Documents/potc/src$ sed -i 's/__m512i/ivec/g' gen_intel_visitor.h 
markus@markus-notebook:~/Documents/potc/src$ sed -i 's/__mmask8/bvec/g' gen_intel_visitor.h 
markus@markus-notebook:~/Documents/potc/src$ sed -i 's/_mm512_\(.*?\)_si512/ivec::\1/g' gen_intel_visitor.h 
markus@markus-notebook:~/Documents/potc/src$ sed -i 's/_mm512_\(.*?\)_epi32/ivec::\1/g' gen_intel_visitor.h 
markus@markus-notebook:~/Documents/potc/src$ sed -i 's/_mm512_\(.*?\)_pd/fvec::\1/g' gen_intel_visitor.h 
markus@markus-notebook:~/Documents/potc/src$ sed -i 's/_mm512_k\(.*?\)(/bvec::k\1(/g' gen_intel_visitor.h 

