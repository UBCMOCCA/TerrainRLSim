#pragma once
#include "learning/NeuralNetTrainer.h"
#include "learning/RecurrentNet.h"

class cRNNTrainer : public cNeuralNetTrainer
{
public:
	
	cRNNTrainer();
	virtual ~cRNNTrainer();

	virtual void Init(const tParams& params);
	virtual void Clear();
	virtual void Reset();

	virtual int AddTuple(const tExpTuple& tuple, int prev_id, int learner_id);
	virtual int AddTuples(const std::vector<tExpTuple>& tuples, int prev_id, int learner_id);

	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

protected:
	struct tSequenceEntry
	{
		int mNextID;
		tSequenceEntry();
	};

	struct tStream
	{
		int mStreamID;
		int mCurrID;
		tStream();
	};

	std::vector<tSequenceEntry> mSequenceBuffer;
	std::vector<int> mSeqHeads;
	std::vector<tStream> mStreams;

	virtual void InitSequenceBuffer();
	virtual void InitStreams();

	virtual void BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size);
	virtual void FetchMinibatch(int size, std::vector<int>& out_batch);
	virtual int GetNumStreams() const;
	virtual int GetNumTuplesPerBatch() const;
	
	virtual int GetProblemXSize() const;
	virtual int GetProblemYSize() const;
	virtual void BuildProblemX(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids,
								const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);

	virtual int IncBufferHead(int head) const;
	virtual void UpdateSequenceBuffer(int t, int prev_id);
	virtual int IncSeqID(int id) const;
	virtual int GetNumSequences() const;
	virtual int GetRandSeqHead() const;

	virtual void UpdateStreams(int id);
	virtual void AdvanceStream(tStream& out_stream);

	virtual bool IsStart(int t) const;
};