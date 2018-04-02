#include "DrawMusculotendonUnit.h"
#include "DrawUtil.h"
#include "sim/MusculotendonUnit.h"

void cDrawMusculotendonUnit::Draw(const cMusculotendonUnit& mtu, double r)
{
	const tVector tendon_col = tVector(0.5, 0.5, 0.5, 0.5);
	const tVector ce_col0 = tVector(0.25, 0, 0, 0.5);
	const tVector ce_col1 = tVector(1, 0, 0, 0.5);
	const double min_ce_len = 0.001;

	int num_pts = mtu.GetNumAttachPts();
	if (num_pts > 1)
	{
		double a = mtu.GetActivation();
		double col_lerp = cMathUtil::Clamp(a, 0.0, 1.0);
		tVector ce_col = (1 - a) * ce_col0  + a * ce_col1;

		tVector prev_pos = mtu.CalcAttachPtWorldPos(0);

		double total_len = mtu.CalcLength();
		double ce_len = mtu.GetCELength();
		double opt_ce_len = mtu.GetOptCELength();
		double tendon_len = 0.5 * (total_len - ce_len);

		int p = 1;
		double curr_len = 0;

		int seg_id = 0;
		double seg_end_len = tendon_len;
		while (p < num_pts)
		{
			tVector pos = mtu.CalcAttachPtWorldPos(p);
			double len = (pos - prev_pos).norm();
			double clamped_len = len;

			bool end_seg = (curr_len + len > seg_end_len);
			if (end_seg)
			{
				clamped_len = seg_end_len - curr_len;
			}

			double lerp = clamped_len / len;
			tVector beg_pos = prev_pos;
			tVector end_pos = beg_pos + lerp * (pos - prev_pos);

			tVector col;
			if (seg_id == 1)
			{
				col = ce_col;
			}
			else
			{
				col = tendon_col;
			}

			double curr_r = (seg_id == 1) ? ((opt_ce_len + min_ce_len) / (ce_len + min_ce_len)) : 1;
			curr_r *= r;

			cDrawUtil::SetLineWidth(curr_r);
			cDrawUtil::SetColor(col);
			//cDrawUtil::DrawLine(beg_pos, end_pos);

			cDrawUtil::PushMatrix();
			const tVector up = tVector(0, 1, 0, 0);
			tVector dir = (end_pos - beg_pos).normalized();
			tMatrix rot = cMathUtil::DirToRotMat(dir, up);
			
			cDrawUtil::Translate(0.5 * (end_pos + beg_pos));
			cDrawUtil::GLMultMatrix(rot);
			cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
			double capsule_h = (end_pos - beg_pos).norm();
			cDrawUtil::DrawCapsule(capsule_h, curr_r);
			cDrawUtil::PopMatrix();

			if (end_seg)
			{
				++seg_id;
				seg_end_len = (seg_id == 1) ? (tendon_len + ce_len) : std::numeric_limits<double>::infinity();
				assert(seg_id < 3);
			}
			else
			{
				++p;
			}
			curr_len += lerp * len;
			prev_pos = end_pos;
		}
	}
}