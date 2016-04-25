#include "AppleTree.h"

#include <iostream>

using namespace std;

// Functions for building a tree
void insert(TreeNode* node, string code, int val){
	//DLOG(INFO) << code << ":" << val << endl;

	node->min = min(node->min, val);
	node->max = max(node->max, val);	
	if (code.length() <= 0){ return; }

	if (code[0] == '0'){
		if (!node->left_node){
			node->left_node = new TreeNode{NULL, NULL, val, val};
		}
		insert(node->left_node, code.substr(1,string::npos), val);
		return;
	} else if (code[0] == '1'){
		if (!node->right_node){
			node->right_node = new TreeNode{NULL, NULL, val, val};
		}
		insert(node->right_node, code.substr(1,string::npos), val);
		return;
	} else {
		throw runtime_error("bad code:"+code);
	}
}

void print_tree(TreeNode* node){
	if (!node) return;

	DLOG(INFO) << "[tree]min:" << node->min << ", max:" << node->max << ", left(0):" << node->left_node << ", right(1):" << node->right_node << endl;
	print_tree(node->left_node);
	print_tree(node->right_node);
}

void AppleJuice::ReadLookup_table(std::string filename){
	// Writes the lookup_table in Chessboard
	TreeNode* head = chessboard.lookup_table;
	head = new TreeNode{NULL, NULL, -1, -1};
	ifstream file ( filename );
	string code;
	string val;
	for (int i = 0; i < 6; i++){
		getline( file, code, ',');
		getline( file, val );

		insert(head, code, stoi(val));
		print_tree(head);
		DLOG(INFO) << endl;
	}
}
