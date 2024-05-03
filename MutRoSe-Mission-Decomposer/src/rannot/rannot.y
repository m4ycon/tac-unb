%define api.prefix {zz}
// Emitted in the header file, after the definition of YYSTYPE.
%code provides
{
  // Tell Flex the expected prototype of yylex.
  #define YY_DECL                             \
    int zzlex ()

  // Declare the scanner.
  YY_DECL;
}

%{
    #include <set>
    #include <map>
    #include <vector>
    #include <iostream>
    #include <string>

    #include "../annotmanager/annotmanager.hpp"

    using namespace std;

    extern int zzlex();
	extern int zzparse();
	extern const char *zzin;
	char* current_parser_file_name2;

    void zzerror(const char *s);
%}

%locations
%define parse.error verbose
%define parse.lac full

%locations

%union {
	bool bval;
	int ival;
	float fval;
	char *sval;
    std::vector<std::string>* vstring;
    general_annot* annot;
}

%token KEY_END_ID
%token <sval> KEY_SEQ KEY_PAR KEY_FALLBACK
%token <sval> STRNAME 

%type <annot> annot
%type <annot> input
%type <annot> expr
%type <annot> expr-no-pt
%type <annot> name-no-pt
%type <annot> expr-or-null
%type <annot> expr-seq
%type <annot> expr-par
%type <annot> expr-fallback
%type <vstring> STRNAME-list

%left KEY_SEQ
%left KEY_PAR
%left KEY_FALLBACK

%start input

%%
//Filling the map up
input: STRNAME KEY_END_ID STRNAME-list annot {$$ = $4; $$->related_goal = $1; goals_and_rannots[$1] = $$;}

annot: '[' expr-or-null ']' {$$ = $2;} | {$$ = new general_annot();}

expr-or-null: expr {$$ = $1;}
            | {$$ = new general_annot();}

expr:  '(' expr-no-pt ')' {$$ = $2;}
    | expr-no-pt {$$ = $1;}
    | name-no-pt {$$ = $1;}

expr-no-pt: expr-seq {$$ = $1;}
          | expr-par {$$ = $1;}
          | expr-fallback {$$ = $1;}

expr-seq: expr KEY_SEQ expr {
    $$ = new general_annot();

    $$->type = OPERATOR;

    $$->content = ";";

    std::vector<general_annot*> children;

    bool unwind = true;

    if($1->type == OPERATOR) {
        if($1->content != ";") {
            unwind = false;
        }
    }

    if($3->type == OPERATOR && unwind) {
        if($3->content != ";") {
            unwind = false;
        }
    } else if($3->type != OPERATOR && unwind) {
        if($1->type != OPERATOR) {
            unwind = false;
        }
    }
    
    if(unwind) {
        if($1->type == OPERATOR) {
            for(general_annot* child : $1->children) {
                children.push_back(child);
            }
        } else {
            children.push_back($1);
        }

        if($3->type == OPERATOR) {
            for(general_annot* child : $3->children) {
                children.push_back(child);
            }
        } else {
            children.push_back($3);
        }
    } else {
        children.push_back($1);
        children.push_back($3);
    }

    $$->children = children;
}

expr-par: expr KEY_PAR expr {
    $$ = new general_annot();

    $$->type = OPERATOR;

    $$->content = "#";

    std::vector<general_annot*> children;

    bool unwind = true;

    if($1->type == OPERATOR) {
        if($1->content != "#") {
            unwind = false;
        }
    }

    if($3->type == OPERATOR && unwind) {
        if($3->content != "#") {
            unwind = false;
        }
    } else if($3->type != OPERATOR && unwind) {
        if($1->type != OPERATOR) {
            unwind = false;
        }
    }
    
    if(unwind) {
        if($1->type == OPERATOR) {
            for(general_annot* child : $1->children) {
                children.push_back(child);
            }
        } else {
            children.push_back($1);
        }

        if($3->type == OPERATOR) {
            for(general_annot* child : $3->children) {
                children.push_back(child);
            }
        } else {
            children.push_back($3);
        }
    } else {
        children.push_back($1);
        children.push_back($3);
    }

    $$->children = children;
}

expr-fallback: KEY_FALLBACK '(' expr ',' expr ')' {
    $$ = new general_annot();

    $$->type = OPERATOR;

    $$->content = "FALLBACK";

    std::vector<general_annot*> children;

    bool unwind = true;

    if($3->type == OPERATOR) {
        if($3->content != "FALLBACK") {
            unwind = false;
        }
    }

    if($5->type == OPERATOR && unwind) {
        if($5->content != "FALLBACK") {
            unwind = false;
        }
    } else if($5->type != OPERATOR && unwind) {
        if($3->type != OPERATOR) {
            unwind = false;
        }
    }
    
    if(unwind) {
        if($3->type == OPERATOR) {
            for(general_annot* child : $3->children) {
                children.push_back(child);
            }
        } else {
            children.push_back($3);
        }

        if($5->type == OPERATOR) {
            for(general_annot* child : $5->children) {
                children.push_back(child);
            }
        } else {
            children.push_back($5);
        }
    } else {
        children.push_back($3);
        children.push_back($5);
    }

    $$->children = children;
}

name-no-pt: STRNAME {
    $$ = new general_annot();

    if($1[0] == 'G') {
        $$->type = GOAL;
    } else {
        $$->type = TASK;
    }

    $$->content = $1;
}

STRNAME-list: STRNAME-list STRNAME {string s($2); free($2); $$->push_back(s);}
			|  {$$ = new vector<string>();}
%%

void set_input_string(const char* in);
void end_lexical_scan(void);

int parse_string(const char* in) {
  set_input_string(in);
  int rv = zzparse();
  end_lexical_scan();
  return rv;
}

void zzerror(const char *s) {
  //cout << "\x1b[31mParse error\x1b[0m in file " << current_parser_file_name2 << " in line \x1b[1m" << yylloc.first_line << "\x1b[0m" << endl;
  if (strlen(s) >= 14 && (strncmp("syntax error, ",s,14) == 0)){
    s += 14;
  }
  cout << "\x1b[31mParse error\x1b[0m in runtime annotation: " << s << endl; 
  //cout << s << endl;
  // might as well halt now:
  exit(-1);
}
