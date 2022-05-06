#include "StraightOptimizer2/TransformPasses.h"
#include "StraightOptimizer2/Transformer.hpp"
#include "StraightOptimizer2/RouteRange.h"

namespace Optimizer2 {
	void TransformPasses::eliminateIMPLICIT_DEF( const Function& function ) {
		for( const auto basic_block : function.getBasicBlocks() ) {
			for( const auto instruction : basic_block->getInstructions() ) {
				if( instruction->isInArgRegion() ) {
				}
				if( instruction->isIMPLICIT_DEF() ) {
					instruction->changeToNOP();
				}
			}
		}
	}

	void TransformPasses::eliminateCOPY( const Function& function ) {
		for( const auto basic_block : function.getBasicBlocks() ) {
			for( const auto instruction : basic_block->getInstructions() ) {
				if( instruction->isCOPY() ) {
					Transformer::shortCutRegOperandsAll( instruction );
				}
			}
		}
		Transformer::eraseInstructions_if( function, []( const auto& instr ) { return instr->isCOPY(); } );
	}

	std::vector<exempt_ptr<Instruction>> TransformPasses::createPhiList( const BasicBlock& bb ) {
		std::vector<exempt_ptr<Instruction>> phiList;
		for( const auto instr : bb.getInstructions() ) {
			if( instr->isPhi() ) {
				assert( instr->getRegOperands().size() == bb.getPredBlocks().size() );
				phiList.push_back( instr );
			}
		}
		return phiList;
	}

	void TransformPasses::eliminatePHI( const Function& function ) {
		for( const auto bb : function.getBasicBlocks() ) {
			const auto phiList = createPhiList( *bb );
			if( phiList.empty() ) { continue; }

			auto insert_positions = Optimizer2::TransformPasses::createFixedRegionBase( *bb );
			for( const auto phi : phiList ) {
				for( std::size_t i = 0; i < bb->getPredBlocks().size(); ++i ) {
					const auto operand = phi->getRegOperand( i );
					insert_positions[i] = Transformer::insertRelayRMOVBefore( insert_positions[i], operand, 'F' );
					const auto rmov = insert_positions[i].getIns();
					Transformer::changeOperandToRelayRMOV( rmov, operand );
				}
			}
		}
	}


	exempt_ptr<RegOperand> TransformPasses::findLongDistanceOperand( const Function& function ) {
		for( const auto basic_block : function.getBasicBlocks() ) {
			for( const auto instruction : basic_block->getInstructions() ) {
				if( instruction->isPhi() ) { continue; }
				for( const auto operand : instruction->getRegOperands() ) {
					if( operand->getDistanceThroughPhi() > MaxDistance ) {
						return operand;
					}
				}
			}
		}
		return make_exempt<RegOperand>( nullptr );
	}

	exempt_ptr<Instruction> TransformPasses::findNearestRelayRMOV( exempt_ptr<RegOperand> operand ) {
		for( const auto instr : ReverseRouteRange( operand ) ) {
			if( instr == operand->getConsumer() ) { continue; }
			if( instr->isLimitRMOV() && instr->getOnlyOneRegOperand()->getProducer() == operand->getProducer() ) {
				return instr;
			}
		}
		return make_exempt<Instruction>( nullptr );
	}

	exempt_ptr<Instruction> TransformPasses::findLongDistanceLimitRMOV( exempt_ptr<BasicBlock> bb ) {
		return bb->getInstructions().find_if( []( const auto &instr ) {
			return instr->isLimitRMOV() && instr->getOnlyOneRegOperand()->getDistanceThroughPhi() > MaxDistance;
		} );
	}

	void TransformPasses::changeOperandToLimitRMOV( exempt_ptr<RegOperand> operand ) {
		const auto rmov = findNearestRelayRMOV( operand );
		if( rmov && rmov->getOnlyOneRegOperand()->getDistanceThroughPhi() <= MaxDistance ) {
			Transformer::changeOperandToRelayRMOV( rmov, operand );
		} else {
			const auto insert_pos = InstructionIterator( operand->getConsumer() ).getFixedRegionTop();
			const auto rmov = Transformer::insertRelayRMOVBefore( insert_pos, operand, 'L' ).getIns();
			Transformer::changeOperandToRelayRMOV( rmov, operand );
			if( operand->getDistanceThroughPhi() > MaxDistance ) {
				assert( !"too large FixedRegion!" );
			}
		}
	}


	void TransformPasses::distanceLimitOnlyLimitRMOV( const Function& function ) {
		bool change;
		do {
			change = false;
			for( const auto basic_block : function.getBasicBlocks() ) {
				while( const auto rmov = findLongDistanceLimitRMOV( basic_block ) ) {
					const auto operand =rmov->getOnlyOneRegOperand();
					while( operand->getDistanceThroughPhi() > MaxDistance ) {
						if( const auto limit_rmov = findNearestRelayRMOV( operand ) ) {
							Transformer::changeOperandToRelayRMOV( limit_rmov, operand );
						} else {
							Transformer::moveBefore( rmov );
							change = true;
						}
					}
				}
			}
		} while( change );
	}

	void TransformPasses::distanceLimit( const Function& function ) {
		while( const auto longDistanceOperand = findLongDistanceOperand( function ) ) {
			changeOperandToLimitRMOV( longDistanceOperand );
			distanceLimitOnlyLimitRMOV( function );
		}
	}

	std::vector<InstructionIterator> TransformPasses::createFixedRegionBase( const BasicBlock& bb ) {
		std::vector<InstructionIterator> ret;
		for( auto incoming_bb : bb.getPredBlocks() ) {
			const auto last_instr = incoming_bb->getInstructions().back();
			const auto insert_pos = InstructionIterator( last_instr );
			if( last_instr->isImplicitFallthrough() ) {
				ret.push_back( Transformer::insertNOPBefore( insert_pos, -1 ) );
			} else {
				ret.push_back( insert_pos );
			}
		}
		return ret;
	}


} // namespace Optimizer2
