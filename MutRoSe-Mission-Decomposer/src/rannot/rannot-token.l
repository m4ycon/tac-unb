%option prefix="zz"

%{
	#include <iostream>
	#include <vector>
    #include "../annotmanager/annotmanager.hpp"
	#include "rannot.hpp"

	using namespace std;
%}

%option noyywrap

%%
\(				{return '(';}
\)				{return ')';}
\[              {return '[';}
\]              {return ']';}
,               {return ',';}
:               {return KEY_END_ID;}
;               {return KEY_SEQ;}
#               {return KEY_PAR;}
FALLBACK        {return KEY_FALLBACK;}

[ \t\n\r]         ; // whitespace

[a-zA-Z][a-zA-Z0-9]*	{zzlval.sval = strdup(yytext); return STRNAME;}
%%

void set_input_string(const char* in) {
  yy_scan_string(in);
}

void end_lexical_scan(void) {
    yy_delete_buffer(YY_CURRENT_BUFFER);
}