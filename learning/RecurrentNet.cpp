#include "RecurrentNet.h"

cRecurrentNet::cRecurrentNet()
{
}

cRecurrentNet::~cRecurrentNet()
{
}

int cRecurrentNet::GetInputSize() const
{
	return cNeuralNet::GetInputSize() - 1; // first input is sequence start indicator
}

void cRecurrentNet::Eval(const Eigen::VectorXd& x, bool is_start, Eigen::VectorXd& out_y) const
{
	const int input_size = GetInputSize();
	assert(HasNet());
	assert(x.size() == input_size);

	tNNData* input_data = GetInputDataX();
	
	double start_val = (is_start) ? 0 : 1;
	input_data[0] = start_val;
	bool valid_offset_scale = ValidOffsetScale();
	for (int i = 0; i < input_size; ++i)
	{
		double val = x[i];
		if (valid_offset_scale)
		{
			val = mInputScale[i] * (val + mInputOffset[i]);
		}
		input_data[i + 1] = val;
	}

	const std::vector<caffe::Blob<tNNData>*>& result_arr = mNet->Forward();
	FetchOutput(result_arr, out_y);
}

int cRecurrentNet::GetNumStreams() const
{
	int num_streams = 1;
	if (HasSolver())
	{
		const auto& input_blobs = GetTrainInputs();
		const auto& input_blob = (*input_blobs)[gXBlobIdx];
		num_streams = input_blob->shape(1);
	}
	return num_streams;
}

int cRecurrentNet::GetProblemXSize() const
{
	int num_streams = GetNumStreams();
	int input_size = GetInputSize();
	return num_streams + num_streams * input_size;
}

int cRecurrentNet::GetProblemYSize() const
{
	int num_streams = GetNumStreams();
	int output_size = GetOutputSize();
	return num_streams * output_size;
}

void cRecurrentNet::LoadTrainData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y)
{
	int batch_size = GetBatchSize();
	int num_batches = static_cast<int>(X.rows()) / batch_size;
	assert(num_batches == 1);

	int num_data = batch_size;
	int data_dim = static_cast<int>(X.cols());
	int label_dim = static_cast<int>(Y.cols());

	int num_streams = GetNumStreams();
	int input_size = GetInputSize();
	int output_size = GetOutputSize();

	assert(data_dim == GetProblemXSize());
	assert(label_dim == GetProblemYSize());

	tNNData* x_data = GetTrainInputDataX();
	tNNData* y_data = GetTrainInputDataY();

	for (int i = 0; i < num_data; ++i)
	{
		auto curr_x = X.row(i);
		auto curr_y = Y.row(i);

		int input_offset = i * data_dim;
		int output_offset = i * label_dim;

		for (int s = 0; s < num_streams; ++s)
		{
			int stream_offset = s * (input_size + 1);
			x_data[input_offset + stream_offset] = curr_x[stream_offset];

			for (int j = 0; j < input_size; ++j)
			{
				double val = curr_x[stream_offset + 1 + j];
				if (ValidOffsetScale())
				{
					val += mInputOffset[j];
					val = val * mInputScale[j];
				}
				x_data[input_offset + stream_offset + 1 + j] = val;
			}

			for (int j = 0; j < output_size; ++j)
			{
				double val = curr_y[s * output_size + j];
				if (ValidOffsetScale())
				{
					val += mOutputOffset[j];
					val = val * mOutputScale[j];
				}
				y_data[output_offset + s * output_size + j] = val;
			}
		}
	}
}
