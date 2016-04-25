#include "AppleTree.h"

#include <iostream>

using namespace std;

void AppleJuice::ReadLookup_table(std::string filename){
	// Writes the lookup_table in Chessboard
	TreeNode* head = chessboard.lookup_table;
	ifstream file ( filename );
	string value;
	getline( file, value, ',' );
	cout << value << endl;
	//head.build_tree()
}
