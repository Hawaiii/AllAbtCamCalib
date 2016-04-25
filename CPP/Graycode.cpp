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

std::pair<float, float> min_max2center_range(int min, int max){
	return make_pair(1.0*(max+min)/2, 1.0*(max-min)/2 );
}

std::pair<float, float> search(TreeNode* tree, std::string s){
	if (!tree or (s.length() == 0 and tree->min == -1)){
		return make_pair(-1, -1);
	}

	int len = s.length();
	if (len == 0) return min_max2center_range(tree->min, tree->max);
	if (!(tree->left_node) and !(tree->right_node)){
		DLOG(INFO) << "Search reaches end of tree but still search code " << s << endl;
		return min_max2center_range(tree->min, tree->max);
	}

	if (s[0] == '0')
		return search(tree->left_node, s.substr(1,string::npos));
	else if (s[0] == '1')
		return search(tree->right_node, s.substr(1,string::npos));
	else
		throw runtime_error("bad search string:" + s);
}


void print_tree(TreeNode* node){
	if (!node) return;

	DLOG(INFO) << "[tree]min:" << node->min << ", max:" << node->max << ", left(0):" << node->left_node << ", right(1):" << node->right_node << endl;
	print_tree(node->left_node);
	print_tree(node->right_node);
}

void readLookup_table(TreeNode* head, std::string filename){
	head = new TreeNode{NULL, NULL, -1, -1};
	ifstream file ( filename );
	string code;
	string val;
	for (int i = 0; i < 6; i++){
		getline( file, code, ',');
		getline( file, val );

		insert(head, code, stoi(val));
	}
	print_tree(head);
	DLOG(INFO) << endl;
}

void AppleJuice::ReadLookup_table(const opt options){
	DLOG(INFO) << "Reading lookup table for x ..." << endl;
	readLookup_table(chessboard.lookup_table_x, options.Lookup_table_dir_x);
	DLOG(INFO) << "Reading lookup table for y ..." << endl;
	readLookup_table(chessboard.lookup_table_y, options.Lookup_table_dir_y);
}

std::pair<cv::Point3f, cv::Point3f> AppleJuice::SearchPoints(std::string xs, std::string ys){
	// Returns the center and confidence of the located 3D point
	//if (!chessboard) throw runtime_error("Cannot SearchPoints because Chessboard is none!");
	if (!(chessboard.lookup_table_x) or !(chessboard.lookup_table_y))
		throw runtime_error("Cannot Search points because lookup tables are missing!");

	pair<float, float> x_pt = search(chessboard.lookup_table_x, xs);
	pair<float, float> y_pt = search(chessboard.lookup_table_y, ys);

	cv::Point3f pt(x_pt.first, y_pt.first, 0);
	cv::Point3f conf(x_pt.second, y_pt.second, 0);
	return make_pair(pt, conf);
}
