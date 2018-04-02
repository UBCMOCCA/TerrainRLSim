#include "RNNTrainer.h"
#include "RNNLearner.h"

cRNNTrainer::tSequenceEntry::tSequenceEntry()
{
	mNextID = gInvalidIdx;
}

cRNNTrainer::tStream::tStream()
{
	mStreamID = gInvalidIdx;
	mCurrID = gInvalidIdx;
}

cRNNTrainer::cRNNTrainer()
{
}

cRNNTrainer::~cRNNTrainer()
{
}

void cRNNTrainer::Init(const tParams& params)
{
	cNeuralNetTrainer::Init(params);
	InitSequenceBuffer();
	InitStreams();
}

void cRNNTrainer::Clear()
{
	cNeuralNetTrainer::Clear();
	mSequenceBuffer.clear();
	mSeqHeads.clear();
	mStreams.clear();
}

void cRNNTrainer::Reset()
{
	cNeuralNetTrainer::Reset();
	InitSequenceBuffer();
	InitStreams();
}

int cRNNTrainer::AddTuple(const tExpTuple& tuple, int prev_id, int learner_id)
{
	int id = cNeuralNetTrainer::AddTuple(tuple, prev_id, learner_id);
	if (id != gInvalidIdx)
	{
		UpdateSequenceBuffer(id, prev_id);
		UpdateStreams(id);
	}
	return id;
}

int cRNNTrainer::AddTuples(const std::vector<tExpTuple>& tuples, int prev_id, int learner_id)
{
	assert(false); // unsupported method use AddTuple() instead
	return gInvalidIdx;
}

void cRNNTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cRNNLearner>(new cRNNLearner(shared_from_this()));
}

void cRNNTrainer::InitSequenceBuffer()
{
	mSeqHeads.clear();
	mSequenceBuffer.clear();
	mSequenceBuffer.resize(mParams.mPlaybackMemSize);
}

void cRNNTrainer::InitStreams()
{
	int num_streams = GetNumStreams();
	mStreams.clear();
	mStreams.resize(num_streams);

	for (int s = 0; s < num_streams; ++s)
	{
		tStream& stream = mStreams[s];
		stream.mStreamID = s;
	}
}

void cRNNTrainer::BuildNetPool(const std::string& net_file, const std::string& solver_file,
								int pool_size)
{
	assert(pool_size > 0);
	pool_size = std::max(1, pool_size);
	mNetPool.clear();
	mNetPool.resize(pool_size);

	for (int i = 0; i < pool_size; ++i)
	{
		auto& net = mNetPool[i];
		net = std::unique_ptr<cRecurrentNet>(new cRecurrentNet());
		net->LoadNet(net_file);
		net->LoadSolver(solver_file);
	}
}

void cRNNTrainer::FetchMinibatch(int size, std::vector<int>& out_batch)
{
	out_batch.resize(size);

	int num_streams = GetNumStreams();
	for (int i = 0; i < size; ++i)
	{
		int s = i % num_streams;
		tStream& stream = mStreams[s];
		AdvanceStream(stream);

		int t = stream.mCurrID;
		assert(t != gInvalidIdx);
		out_batch[i] = t;
	}
}

int cRNNTrainer::GetNumStreams() const
{
	const auto& curr_net = GetCurrNet();
	cRecurrentNet* rnn = static_cast<cRecurrentNet*>(curr_net.get());
	return rnn->GetNumStreams();
}

int cRNNTrainer::GetNumTuplesPerBatch() const
{
	return GetBatchSize() * GetNumStreams();
}

int cRNNTrainer::GetProblemXSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetProblemXSize();
}

int cRNNTrainer::GetProblemYSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetProblemYSize();
}

void cRNNTrainer::BuildProblemX(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	int batch_size = GetBatchSize();
	int num_streams = GetNumStreams();
	int input_size = GetInputSize();

	assert(num_data == batch_size * num_streams);
	assert(out_prob.mX.rows() == batch_size);
	
	for (int b = 0; b < batch_size; ++b)
	{
		auto curr_row = out_prob.mX.row(b);
		for (int s = 0; s < num_streams; ++s)
		{
			int i = b * num_streams + s;
			int t = tuple_ids[i];
			tExpTuple tuple = GetTuple(t);

			Eigen::VectorXd x;
			BuildTupleX(tuple, x);

			bool is_start = IsStart(t);

			int curr_offset = s * (input_size + 1);
			curr_row(curr_offset) = (is_start) ? 0 : 1;
			curr_row.segment(curr_offset + 1, input_size) = x;
		}
	}
}

void cRNNTrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids,
								const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	int batch_size = GetBatchSize();
	int num_streams = GetNumStreams();
	int output_size = GetOutputSize();

	assert(num_data == GetNumTuplesPerBatch());
	assert(out_prob.mY.rows() == GetBatchSize());

	for (int b = 0; b < batch_size; ++b)
	{
		auto row = out_prob.mY.row(b);
		for (int s = 0; s < num_streams; ++s)
		{
			int i = b * num_streams + s;
			int t = tuple_ids[i];
			tExpTuple tuple = GetTuple(t);

			Eigen::VectorXd y;
			BuildTupleY(net_id, tuple, y);
			row.segment(s * output_size, output_size) = y;
		}
	}
}

int cRNNTrainer::IncBufferHead(int head) const
{
	int buffer_size = GetPlaybackMemSize();
	int next = gInvalidIdx;
	int num_tuples = mExpBuffer->GetNumTuples();

	if (num_tuples < buffer_size)
	{
		next = num_tuples;
	}
	else
	{
		assert(head != gInvalidIdx);
		const tSequenceEntry& entry = mSequenceBuffer[head];
		next = entry.mNextID;
		if (next == gInvalidIdx)
		{
			if (mSeqHeads.size() > 0)
			{
				next = mSeqHeads[0];
			}
			else
			{
				next = 0;
			}
		}
	}
	return next;
}

void cRNNTrainer::UpdateSequenceBuffer(int t, int prev_id)
{
	auto heads_beg = mSeqHeads.begin();
	auto heads_end = mSeqHeads.end();
	auto heads_iter = std::find(heads_beg, heads_end, t);

	bool contains = heads_iter != heads_end;
	if (contains)
	{
		mSeqHeads.erase(heads_iter);
	}

	bool is_start = prev_id == gInvalidIdx;
	if (is_start)
	{
		mSeqHeads.push_back(t);
	}
	else
	{
		mSequenceBuffer[prev_id].mNextID = t;
	}
	mSequenceBuffer[t].mNextID = gInvalidIdx;
}

int cRNNTrainer::IncSeqID(int id) const
{
	int next_id = gInvalidIdx;
	if (id != gInvalidIdx)
	{
		const tSequenceEntry& entry = mSequenceBuffer[id];
		next_id = entry.mNextID;
	}

	bool end_seq = (next_id == gInvalidIdx);
	if (end_seq)
	{
		next_id = GetRandSeqHead();
	}
	return next_id;
}

int cRNNTrainer::GetNumSequences() const
{
	return static_cast<int>(mSeqHeads.size());
}

int cRNNTrainer::GetRandSeqHead() const
{
	int rand_head = gInvalidIdx;
	int num_seqs = GetNumSequences();
	if (num_seqs > 0)
	{
		int rand_idx = cMathUtil::RandInt(0, num_seqs);
		rand_head = mSeqHeads[rand_idx];
	}
	else
	{
		printf("Failed to find a valid start to a sequence.\n");
		assert(false);
	}
	return rand_head;
}

void cRNNTrainer::UpdateStreams(int id)
{
	int num_streams = GetNumStreams();
	for (int s = 0; s < num_streams; ++s)
	{
		tStream stream = mStreams[s];
		if (stream.mCurrID == id)
		{
			stream.mCurrID = gInvalidIdx;
		}
	}
}

void cRNNTrainer::AdvanceStream(tStream& out_stream)
{
	out_stream.mCurrID = IncSeqID(out_stream.mCurrID);
}

bool cRNNTrainer::IsStart(int t) const
{
	int flags = mExpBuffer->GetFlags(t);
	return tExpTuple::TestFlag(flags, tExpTuple::eFlagStart);
}
