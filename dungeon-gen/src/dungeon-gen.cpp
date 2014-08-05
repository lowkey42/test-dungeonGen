//============================================================================
// Name        : dungeon-gen.cpp
// Author      : lowkey
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <cassert>

#include "DungeonGenerator.hpp"

std::ostream& operator<<(std::ostream& os, TileType tile) {
	switch( tile ) {
		case TileType::NOTHING:
			os<<" ";
			break;

		case TileType::WALL:
			os<<"#";
			break;

		case TileType::FLOOR:
			os<<".";
			break;

		case TileType::STAIRS_DOWN:
			os<<"<";
			break;

		case TileType::STAIRS_UP:
			os<<">";
			break;

		case TileType::DOOR:
			os<<"+";
			break;

		case TileType::BARRIER:
			os<<"X";
			break;
	}

	return os;
}

int main() {
	std::cout << "" << std::endl; // prints

	std::unique_ptr<Map> map = generate(42, 1);

	for( auto y=0; y<map->height; ++y ) {
		for( auto x=0; x<map->width; ++x ) {
			const auto& tile = map->get(x,y);

			if( tile.type==TileType::FLOOR && tile.roomId!=0 )
				std::cout<<static_cast<char>(tile.roomId%('z'-'a')+'a');
			else
				std::cout<< tile.type;
		}
		std::cout<<std::endl;
	}

	return 0;
}
