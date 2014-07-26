/*
 * DungeonGenerator.hpp
 *
 *  Created on: 08.07.2014
 *      Author: lowkey
 */

#ifndef DUNGEONGENERATOR_HPP_
#define DUNGEONGENERATOR_HPP_

#include <cstdint>
#include <vector>
#include <map>
#include <set>
#include <memory>

struct Tile;
struct Map;

struct Position {
	int32_t x,y;

	Position operator+(const Position& o)const {
		return Position{x+o.x, y+o.y};
	}
	Position operator-(const Position& o)const {
		return Position{x-o.x, y-o.y};
	}
	Position operator/(int factor)const {
		return Position{x/factor, y/factor};
	}
	bool operator==(const Position& o)const {
		return x==o.x && y==o.y;
	}
	bool operator<(const Position& o)const {
		if(x < o.x)
			return true;
		if(x > o.x)
			return false;
		else
			return y < o.y;
	}
};

enum class RoomType {
	NORMAL = 0,
	START,
	END,
	BOSS_ROOM,
	POSITIV_ROOM,
	NEGATIV_ROOM
};

typedef uint32_t RoomId;
struct Room {
	RoomId id;
	const Position position;
	const Position size;
	int8_t doorCount;
	RoomType type;
	Map& map;
	int32_t debug;

	Room(Map& map, Position position, Position size);
	Room(Room&& o);
	Room(const Room&)=delete;
	~Room();

	Room& operator=(const Room&)=delete;
	void addTile(Position pos);
	Position getCenter()const {
		return position + size/2;
	}
	int8_t calcMaxDoors();

	private:
		static RoomId genNextId();

		int8_t maxDoors;
};

enum class TileType {
	NOTHING = 0,

	WALL,
	FLOOR,
	DOOR,
	STAIRS_UP,
	STAIRS_DOWN,

	BARRIER
};
struct Tile {
	TileType type;
	RoomId roomId;
};
struct Map {
	const int16_t height;
	const int16_t width;

	std::vector<Tile> tiles;
	std::map<RoomId, Room> rooms;

	Map(int16_t width, int16_t height) : height(height), width(width) {
		tiles.resize(height*width, Tile{TileType::NOTHING, 0});
	}
	Map(const Map&)=delete;
	Map& operator=(Map&)=delete;

	Tile& get(int16_t x, int16_t y);
	Tile& get(Position pos){
		return get(pos.x,pos.y);
	}
};


extern std::unique_ptr<Map> generate(uint32_t seed, uint8_t level);

#endif /* DUNGEONGENERATOR_HPP_ */
