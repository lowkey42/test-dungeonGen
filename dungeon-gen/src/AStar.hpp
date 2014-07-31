/*
 * AStar.hpp
 *
 *  Created on: 10.07.2014
 *      Author: lowkey
 */

#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include "DungeonGenerator.hpp"

#include <vector>
#include <algorithm>
#include <cassert>
#include <iostream>

typedef std::vector<Position> Path;

template<class CostCalculatorType>
class AStar {
	public:
		AStar(Position limits, CostCalculatorType costCalculator=CostCalculatorType()) : _limits(limits), _costCalculator(costCalculator) {}

		Path searchPath(Position start, Position target);

	private:
		struct Node {
			Position position;
			float costs;

			int32_t prevFieldIndex;
			uint16_t pathLength;

			Node(Position position, float costs, int32_t prevFieldIndex, uint16_t pathLength)
				: position(position), costs(costs), prevFieldIndex(prevFieldIndex), pathLength(pathLength) {}

			bool operator>(const Node& o)const {
				return costs>o.costs;
			}
		};
		typedef std::greater<Node> NodeComp;

		Path buildPath(const Node& targetNode)const;
		void processSuccessor(std::size_t prevIndex, const Node& prevNode, Position posOffset, Position target);


		std::vector<Node> _openList;

		std::map<Position, std::size_t> _closedSet;
		std::vector<Node> _closedList;

		const Position _limits;
		const CostCalculatorType _costCalculator;
};

template<class CostCalculatorType>
inline AStar<CostCalculatorType> createPathFinder(Position limits, CostCalculatorType costCalculator=CostCalculatorType()) {
	return AStar<CostCalculatorType>(limits, costCalculator);
}

template<class CostCalculatorType>
Path AStar<CostCalculatorType>::searchPath(Position start, Position target) {
	_openList.clear();
	_closedList.clear();
	_closedSet.clear();

	_openList.emplace_back(start, 0, -1, 1);
	std::push_heap(_openList.begin(), _openList.end(), NodeComp());

	while( !_openList.empty() ) {
		// add to closed list
		auto fieldIndex = _closedList.size();
		_closedList.push_back(_openList.front());
		auto& field = _closedList.back();
		_closedSet[field.position]=fieldIndex;
		assert( _closedSet.find(field.position)!=_closedSet.end() );

		// remove from open list
		std::pop_heap(_openList.begin(), _openList.end(), NodeComp());
		_openList.pop_back();

		assert(field.costs<=_openList.front().costs);

		// target reached
		if( field.position==target )
			return buildPath(field);

		for( auto i : {-1,1} ) {
			processSuccessor( fieldIndex, field, Position{i,0}, target );
			processSuccessor( fieldIndex, field, Position{0,i}, target );
		}
	}

	return Path();
}

template<class CostCalculatorType>
Path AStar<CostCalculatorType>::buildPath(const Node& targetField)const {
	Path path(targetField.pathLength-1);

	const Node* f = &targetField;
	for(auto i=path.size()-1; f->prevFieldIndex>=0; i--, f=&_closedList[f->prevFieldIndex] ) {
		assert(i>=0);

		path[i] = f->position;
	}

	return path;
}

template<class CostCalculatorType>
void AStar<CostCalculatorType>::processSuccessor(std::size_t prevIndex, const Node& prevNode, Position posOffset, Position target) {
	const Position pos = posOffset+prevNode.position;
	const float costs = prevNode.costs + _costCalculator(
			prevNode.prevFieldIndex>=0 ? _closedList[prevNode.prevFieldIndex].position : prevNode.position,
					prevNode.position,
					pos,
					target);

	if( pos.x<=0 || pos.x>=_limits.x-1 || pos.y<=0 || pos.y>=_limits.y-1 || costs>=10000 )
		return;

	if( _closedSet.find(pos)==_closedSet.end() ) {
		for(std::size_t i=0; i<_openList.size(); ++i) {
			Node& n = _openList[i];

			if( n.position==pos ) {
				// update score and path
				if( costs<n.costs ) {
					n.costs=costs;
					n.prevFieldIndex = prevIndex;
					n.pathLength = prevNode.pathLength+1;

					// reorder heap
					std::make_heap(_openList.begin(), _openList.end(), NodeComp());
				}

				return;
			}
		}

		// not in open or closed list
		_openList.emplace_back(pos, costs, prevIndex, prevNode.pathLength+1);
		std::push_heap(_openList.begin(), _openList.end(), NodeComp());
	}
}

#endif /* ASTAR_HPP_ */
