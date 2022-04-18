#include "StraightOptimizer2/Optimizer2.h"
#include "StraightOptimizer2/Transformer.hpp"

namespace Optimizer2 {
	void Transformer::shortCutRegOperand( exempt_ptr<RegOperand> regOperand ) {
		const auto& middle = regOperand->getProducer()->regOperands[0];

		
		regOperand->producer = middle->getProducer();
		
		regOperand->transitBasicBlocks.insert( regOperand->transitBasicBlocks.end(), middle->transitBasicBlocks.begin() + 1, middle->transitBasicBlocks.end() );

		
		middle->getProducer()->users.insert( regOperand );
	}

	void Transformer::shortCutRegOperandsAll( exempt_ptr<Instruction> instr ) {
		
		for( const auto user_operand : instr->getUsers() ) {
			shortCutRegOperand( user_operand );
		}
		
		instr->getOnlyOneRegOperand()->getProducer()->users.erase( instr->getOnlyOneRegOperand() );
	}

	InstructionIterator Transformer::insertNOPBefore( InstructionIterator insert_pos, int svreg_hint ) {
		
		return BasicBlock::insertBefore( insert_pos, Instruction::createNOP( insert_pos.getBB(), svreg_hint ) );
	}

	InstructionIterator Transformer::insertRelayRMOVBefore( InstructionIterator insert_pos, exempt_ptr<RegOperand> regOperand, char extra ) {
		
		return BasicBlock::insertBefore( insert_pos, Instruction::createRMOV( insert_pos.getBB(), regOperand->getProducer(), extra ) );
	}

	void Transformer::changeOperandToRelayRMOV( exempt_ptr<Instruction> rmov, exempt_ptr<RegOperand> regOperand ) {
		assert( rmov->isRMOV() );
		assert( rmov->getOnlyOneRegOperand()->getProducer() == regOperand->getProducer() );

		const auto phi_target_bb = regOperand->getConsumer()->isPhi() ? regOperand->getTransitBasicBlocks()[1] : exempt_ptr<BasicBlock>( nullptr );

		
		regOperand->getProducer()->users.erase( regOperand );
		
		*regOperand = RegOperand( rmov, regOperand->getConsumer(), phi_target_bb );
		
		rmov->users.insert( regOperand );
	}

	void Transformer::moveBeforeBasicBlock( exempt_ptr<Instruction> instr ) {
		const auto iter = InstructionIterator( instr );
		const auto bb = instr->getParent();
		const auto pred_bb = bb->getOnlyOnePredBlock();
		const auto pred_branch = pred_bb->getBranchInstructionTo( bb );
		const auto insert_pos = InstructionIterator( pred_branch ).getFixedRegionTop();

		
		const auto delete_target = iter.getIterator();
		BasicBlock::insertBefore( insert_pos, std::move( *iter ) );
		bb->eraseInstruction( delete_target );
		instr->parent = pred_bb;

		
		for( const auto operand : instr->getRegOperands() ) {
			operand->pop_consumerBasicBlock();
		}
		
		for( const auto user_operand : instr->getUsers() ) {
			user_operand->push_producerBasicBlock( pred_bb );
		}
	}

	void Transformer::moveBefore( exempt_ptr<Instruction> instr ) {
		const auto iter_instr = InstructionIterator( instr );

		if( iter_instr.isTopOfBasicBlock() ) {
			
			moveBeforeBasicBlock( instr );
		} else {
			const auto insert_pos = (iter_instr - 1).getFixedRegionTop();
			
			
			
		
			std::rotate( insert_pos.getIterator(), iter_instr.getIterator(), iter_instr.getIterator() + 1 );
		}
	}

}
