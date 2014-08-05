/*
 * DungeonGenerator.cpp
 *
 *  Created on: 08.07.2014
 *      Author: lowkey
 */

#include "DungeonGenerator.hpp"

#include "AStar.hpp"

#include <random>
#include <functional>
#include <iostream>
#include <cassert>
#include <array>
#include <cmath>

template<typename T>
struct Range {
	T min;
	T max;
};

struct DungeonCfg {
	Position mapSize;
	Range<Position> roomSize;
	std::size_t rooms;
	bool complexRooms;
	std::size_t smoothRuns;
	bool growRooms;
};

constexpr DungeonCfg DUNGEON_LEVELS[] {
	{{50,50}, {{7,7}, {10,10}}, 10, false, 0, false },
	{{64,64}, {{3,3}, {8,8}}, 50, true, 0, false },

	// ...
	{{64,64}, {{10,10}, {20,20}}, 20, true, 17, true }
};


struct uniform_pos_distribution {
	std::uniform_int_distribution<decltype(Position::x)> distX;
	std::uniform_int_distribution<decltype(Position::x)> distY;

	uniform_pos_distribution(Range<Position> range) : distX(range.min.x, range.max.x), distY(range.min.y, range.max.y) {}

	template<typename _UniformRandomNumberGenerator>
	Position operator()(_UniformRandomNumberGenerator& rng) {
		return Position{distX(rng), distY(rng)};
	}
};

std::size_t countNeighbores( Map& map, Position pos, int range, std::function<bool(const Tile&)> ev ) {
	const auto x = static_cast<int32_t>(pos.x);
	const auto y = static_cast<int32_t>(pos.y);
	std::size_t count = 0;

	for( int32_t yo=-range; yo<=range; ++yo ) {
		for( int32_t xo=-range; xo<=range; ++xo ) {
			if( x+xo>=0 && x+xo<map.width && y+yo>=0 && y+yo<map.height && (xo!=0 || yo!=0) )
				if(ev(map.get(x+xo,y+yo)))
					count++;
		}
	}

	return count;
}
std::size_t countNeighbores( Map& map, Position pos, std::function<bool(const Tile&)> ev ) {
	return countNeighbores(map,pos,1,ev);
}
void visitNeighbores( Map& map, Position pos, int range, std::function<void(Position, Tile&)> visitor ) {
	const auto x = static_cast<int32_t>(pos.x);
	const auto y = static_cast<int32_t>(pos.y);

	for( int32_t yo=-range; yo<=range; ++yo ) {
		for( int32_t xo=-range; xo<=range; ++xo ) {
			if( x+xo>=0 && x+xo<map.width && y+yo>=0 && y+yo<map.height && (xo!=0 || yo!=0) )
				visitor(Position{x+xo,y+yo}, map.get(x+xo,y+yo));
		}
	}
}
void visitNeighbores( Map& map, Position pos, std::function<void(Position, Tile&)> visitor ) {
	visitNeighbores(map,pos,1,visitor);
}


void buildRooms(Map& map, Range<Position> roomSizeRange, std::size_t roomCount, bool joinIntersectingRooms, std::mt19937& rng);
void buildWalls(Map& map);

void smoothRooms(Map& map, std::size_t runs, bool grow);
void buildCorridors(Map& map, std::mt19937& rng);
void assignRoomProperties(Map& map, std::mt19937& rng); //< e.g. boss-room, ...
void placeThings(Map& map, std::mt19937& rng);

std::unique_ptr<Map> generate(uint32_t seed, uint8_t level) {
	const auto& cfg = DUNGEON_LEVELS[level];

	std::unique_ptr<Map> map(new Map(cfg.mapSize.x, cfg.mapSize.y));

	std::mt19937 rng(seed+level);

	buildRooms(*map.get(), cfg.roomSize, cfg.rooms, cfg.complexRooms, rng);

	buildWalls(*map.get());

//	smoothRooms(*map.get(), cfg.smoothRuns, cfg.growRooms);
//
//	buildCorridors(*map.get(), rng);
//
//	assignRoomProperties(*map.get(), rng);
//
//	placeThings(*map.get(), rng);

	return map;
}

void buildWalls(Map& map) {
	for( auto y=0; y<map.height; ++y ) {
		for( auto x=0; x<map.width; ++x ) {
			if( map.get(x,y).type==TileType::NOTHING && countNeighbores(map, Position{x,y}, [](const Tile& t){return t.type==TileType::FLOOR;})>0 ) {
				auto nothingCount = countNeighbores(map, Position{x,y}, [](const Tile& t){return t.type==TileType::NOTHING;});
				if( nothingCount>0 || x==0 || y==0 || x==map.width-1 || y==map.height-1 )
					map.get(x,y).type = TileType::WALL;
			}
		}
	}
}

void buildRooms(Map& map, Range<Position> positionRange, Range<Position> roomSizeRange, std::size_t roomCount, std::bernoulli_distribution joinIntersectingRoomDistr, std::mt19937& rng) {
	uniform_pos_distribution genPos(positionRange);
	uniform_pos_distribution genSize(roomSizeRange);

	std::vector<bool> roomIntersectionAllowed(roomCount);
	for( std::size_t i=0; i<roomCount; i++ )
		roomIntersectionAllowed[i] = joinIntersectingRoomDistr(rng);;

	int triesLeft = 200;

	for( std::size_t i=0; i<roomCount; i++ ) {
		Position roomPos = genPos(rng);
		Position roomSize = genSize(rng);

		if( std::abs(roomSize.x-roomSize.y)>=5 ) {
			if( roomSize.x<roomSize.y )
				roomSize.x+=(roomSize.y-roomSize.x)/2;
			else
				roomSize.y+=(roomSize.x-roomSize.y)/2;
		}

		bool joinThisRoomWithOthers = roomIntersectionAllowed[i];

		auto free = [&](int dmz){
			for( auto y=std::max(0,roomPos.y-dmz); y<std::min((int32_t)map.height, roomSize.y+roomPos.y+dmz*2); ++y )
				for( auto x=std::max(0,roomPos.x-dmz); x<std::min((int32_t)map.width, roomSize.x+roomPos.x+dmz*2); ++x )
					if( map.get(x,y).type!=TileType::NOTHING )
						return false;

			return true;
		};

		if(!joinThisRoomWithOthers && !free(3) ) {
			if( triesLeft>0 ) {
				i--;
				triesLeft--;
				continue;
			} else
				return;
		}

		for( auto y=roomPos.y; y<roomSize.y+roomPos.y; ++y )
			for( auto x=roomPos.x; x<roomSize.x+roomPos.x; ++x )
				map.get(x,y).type = TileType::FLOOR;
	}
}

void buildRooms(Map& map, Range<Position> roomSizeRange, std::size_t roomCount, bool joinIntersectingRooms, std::mt19937& rng) {
	buildRooms(map,
			{{1, 1}, {map.width-1-roomSizeRange.max.x, map.height-1-roomSizeRange.max.y}},
			roomSizeRange, roomCount, std::bernoulli_distribution(joinIntersectingRooms ? 0.5 : 0), rng);
}

void smoothRooms(Map& map, std::size_t runs, bool grow) {
	std::vector<TileType> nextTT(map.tiles.size(), TileType::NOTHING);

		for( std::size_t i=0; i<runs; ++i ) {
			for( auto y=0; y<map.height; ++y ) {
				for( auto x=0; x<map.width; ++x ) {
					auto nothingCount = countNeighbores(map, Position{x,y},2, [](const Tile& t){return t.type==TileType::NOTHING;});
					auto wallCount = countNeighbores(map, Position{x,y}, [](const Tile& t){return t.type==TileType::WALL;});
					auto floorCount = countNeighbores(map, Position{x,y}, [](const Tile& t){return t.type==TileType::FLOOR;});

					switch( map.get(x,y).type ) {
						case TileType::NOTHING:
							if( grow && wallCount>5 )
								nextTT[y*map.width + x] = TileType::WALL;
							break;

						case TileType::WALL:
							if( wallCount+floorCount>7 && nothingCount<5 )
								nextTT[y*map.width + x] = TileType::FLOOR;
							break;
					}
				}
			}

			for( auto y=0; y<map.height; ++y ) {
				for( auto x=0; x<map.width; ++x ) {
					if( nextTT[y*map.width + x]!=TileType::NOTHING ) {
						auto& tile = map.get(x,y);

						RoomId borderRoom = tile.roomId;
						visitNeighbores(map, Position{x,y}, [&](Position tp, Tile& t){
							if( t.type==TileType::NOTHING )
								t.type = TileType::WALL;

							if( t.roomId!=0 && borderRoom==0 )
								borderRoom = t.roomId;
						});

						if( borderRoom!=0 ) {
							tile.type = TileType::FLOOR;
							nextTT[y*map.width + x]=TileType::NOTHING;
							assert(map.rooms.find(borderRoom)!=map.rooms.end());
							auto& borderRoomRef = map.rooms.find(borderRoom)->second;
							borderRoomRef.addTile(Position{x,y});

							visitNeighbores(map, Position{x,y}, [&](Position tp, Tile& t){
								if( t.type==TileType::WALL && countNeighbores(map, tp, [](const Tile& t){return t.type==TileType::NOTHING;})==0 ) {
									t.type=TileType::FLOOR;
								}

								borderRoomRef.addTile(tp);
							});
						}
					}
				}
			}
		}
}

void buildCorridor(Map& map, Path path) {
	int32_t lastDoorDist=9999;
	RoomId lastRoomId = 0;
	for( auto& node : path ) {
		auto& tile = map.get(node);
		lastDoorDist++;

		switch( tile.type ) {
			case TileType::WALL:
				if( tile.roomId!=0 && tile.roomId!=lastRoomId && lastDoorDist>=5 ) {
					lastDoorDist=0;
					tile.type = TileType::DOOR;
					lastRoomId = tile.roomId;

				} else
					tile.type = TileType::FLOOR;

				if( tile.roomId!=0 ) {
					assert(map.rooms.find(tile.roomId)!=map.rooms.end());
					auto room = map.rooms.find(tile.roomId);
					assert(room!=map.rooms.end());

					room->second.doorCount++;
				}
				break;

			case TileType::NOTHING:
				tile.type = TileType::FLOOR;
				break;
		}

		visitNeighbores(map, Position{node.x,node.y}, [&](Position tp, Tile& t){
			if( t.type==TileType::NOTHING )
				t.type = TileType::WALL;
		});
	}
}

template<class T, class FUNCT>
typename std::vector<T>::iterator find_min(std::vector<T>& c, FUNCT weightFunct) {
	typedef typename std::vector<T>::iterator Iter;

	assert(!c.empty());

	if(c.size()==1)
		return c.begin();

	Iter cMin = c.begin();
	auto cWeight = weightFunct(c[0]);
	for(std::size_t i=1; i<c.size(); ++i) {
		auto nw = weightFunct(c[i]);
		if(nw<cWeight) {
			cMin = c.begin() + i;
			cWeight = nw;
		}
	}

	return cMin;
}
template<class T, class FUNCT>
T findAndRemove_min(std::vector<T>& c, FUNCT weightFunct) {
	auto iter = find_min(c, weightFunct);
	T returnValue = std::move(*iter);

	c.erase(iter);
	return returnValue;
}

void buildCorridors(Map& map, std::mt19937& rng) {
//	std::bernoulli_distribution doorProp(0.25);

	auto pathScorer = [&](Position prevPrev, Position prev, Position node, Position goal){
		float result = (std::abs(node.x-goal.x) + std::abs(node.y-goal.y));

		const auto& nodeTile = map.get(node);

		switch(nodeTile.type) {
			case TileType::WALL:
				result+=10;

				if( map.get(prev).type==TileType::WALL )
					result+=400;

				result+= 10* countNeighbores(map,node+(node-prev),1,[](const Tile& t){return t.type==TileType::WALL;}) +
						 10* countNeighbores(map,node-(node-prev),1,[](const Tile& t){return t.type==TileType::WALL;}) +
							2 * countNeighbores(map,node+((node-prev)*2),1,[](const Tile& t){return t.type==TileType::WALL;})+
							1 * countNeighbores(map,node,1,[](const Tile& t){return t.type==TileType::WALL;});


				{
					Position ln{node.x-1, node.y}, rn{node.x+1, node.y}, tn{node.x, node.y-1}, bn{node.x, node.y+1};

					result+= (ln==prev || rn==prev || map.get(ln).type != map.get(rn).type) &&
							 (tn==prev || bn==prev || map.get(tn).type != map.get(bn).type) ? 10000 : 0;
				}

				break;

			case TileType::NOTHING:
				if( countNeighbores(map,node,2,[](const Tile& t){return t.type==TileType::WALL;})>3 )
					result+= (countNeighbores(map,node,2,[](const Tile& t){return t.type==TileType::WALL;})-3) *5;

				if( prevPrev.x!=node.x && prevPrev.y!=node.y )
					result+=2;

				result+=25;
				break;

			default:
				break;
		}

		return result;
	};
	auto pathFinder = createPathFinder(Position{map.width,map.height}, pathScorer);


	// set openRooms = [all rooms]
	std::vector<Room*> openRooms;
	for( auto& r : map.rooms )
		openRooms.push_back(&r.second);

	// set currentRoom = ?
	// remove(currentRoom, openRooms)
	auto currentRoom = findAndRemove_min(openRooms, [](Room* r){return r->getCenter().x+r->getCenter().y;});
	currentRoom->type=RoomType::START;

	const auto closestRoomFunct = [&currentRoom](Room* r){
		auto distX = std::abs(r->getCenter().x-currentRoom->getCenter().x);
		auto distY = std::abs(r->getCenter().y-currentRoom->getCenter().y);

		if( std::abs(distX-distY)<10 )
			return std::min(distX, distY);
		else
			return distX + distY;
	};

	// while( notEmpty(openRooms) )
	while( !openRooms.empty() ) {
		// set nextRoom = findClosestRoom(currentRoom)
		// remove(nextRoom, openRooms)
		auto nextRoom = findAndRemove_min(openRooms, closestRoomFunct);

		//	buildCorridor(currentRoom, nextRoom)
		buildCorridor( map, pathFinder.searchPath(currentRoom->getCenter(), nextRoom->getCenter()) );

		// currentRoom=nextRoom;
		currentRoom = nextRoom;
	}

	if( currentRoom->type!=RoomType::START )
		currentRoom->type=RoomType::END;
}

void assignRoomProperties(Map& map, std::mt19937& rng) {

}

void placeThings(Map& map, std::mt19937& rng) {
	for( auto& r : map.rooms ) {
		switch( r.second.type ) {
			case RoomType::START: {
				map.get(uniform_pos_distribution({r.second.position + Position{1,1}, r.second.position+r.second.size - Position{2,2}})(rng)).type = TileType::STAIRS_UP;
				break;}

			case RoomType::END:
				map.get(uniform_pos_distribution({r.second.position + Position{1,1}, r.second.position+r.second.size - Position{2,2}})(rng)).type = TileType::STAIRS_DOWN;
				break;

			default:
				map.get(r.second.getCenter()).type = TileType::BARRIER;
				break;
		}
	}
}

Tile& Map::get(int16_t x, int16_t y) {
	return tiles[y*width + x];
}

RoomId Room::genNextId() {
	static RoomId id = 0;

	return ++id;
}
Room::Room(Map& map, Position position, Position size) : id(genNextId()), position(position), size(size), doorCount(0), type(RoomType::NORMAL), map(map), maxDoors(0) {
}
Room::Room(Room&& o) : id(o.id), position(o.position), size(o.size), doorCount(o.doorCount), type(o.type), map(o.map), fields(std::move(o.fields)), maxDoors(o.maxDoors) {
	o.id=0;
}
Room::~Room() {
//	if( id!=0 )
//		for( auto tp : fields )
//			map.get(tp).roomId=0;
}

void Room::addTile(Position pos) {
	auto& tile = map.get(pos);
	if( tile.roomId==0 ) {
		fields.insert(pos);
		tile.roomId = id;

	} else if( tile.roomId!=id ) {
		assert(map.rooms.find(tile.roomId)!=map.rooms.end());
		auto otherRoom = map.rooms.find(tile.roomId);
		fields.insert(otherRoom->second.fields.begin(), otherRoom->second.fields.end());
		for( auto& tp : otherRoom->second.fields )
			map.get(tp).roomId = id;

		map.rooms.erase(otherRoom);
	}

	for( auto y=0; y<map.height; ++y )
		for( auto x=0; x<map.width; ++x )
			if( map.get(x,y).roomId==id )
				assert( fields.find(Position{x,y})!=fields.end() );

}

int8_t Room::calcMaxDoors() {
	if( maxDoors>0 )
		return maxDoors;

	uint32_t roomSize=fields.size();

	return maxDoors = static_cast<int8_t>(
			std::log(static_cast<float>(roomSize)) /
			std::log(5.f) -0.5f);
}
