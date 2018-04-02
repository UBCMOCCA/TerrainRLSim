#include "DrawScenarioKinMotion.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"
#include "util/FileUtil.h"

const double gLinkWidth = 0.025f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6f, 0.65f, 0.675f, 1);
const tVector gCamFocus0 = tVector(0, 0.75, 0, 0);
const double gGroundHeight = 0;

static bool gHackMarker = false;

cDrawScenarioKinMotion::cDrawScenarioKinMotion(cCamera& cam)
	: cDrawScenarioTerrainRL(cam)
{
}

cDrawScenarioKinMotion::~cDrawScenarioKinMotion()
{
}

void cDrawScenarioKinMotion::Init()
{
	cDrawScenarioTerrainRL::Init();
	mScene.Init();
}

void cDrawScenarioKinMotion::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenarioTerrainRL::ParseArgs(parser);
	mScene.ParseArgs(parser);

	parser->ParseString("annotate_output_file", mAnnotateOutputFile);
}

void cDrawScenarioKinMotion::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioTerrainRL::Keyboard(key, x, y);

	switch (key)
	{
	case 'd':
		Annotate();
		break;
	}
}

void cDrawScenarioKinMotion::Reset()
{
	cDrawScenarioTerrainRL::Reset();
	mScene.Reset();
}

void cDrawScenarioKinMotion::Clear()
{
	cDrawScenarioTerrainRL::Clear();
	mScene.Clear();
}

std::string cDrawScenarioKinMotion::BuildTextInfoStr() const
{
	const auto& character = mScene.GetCharacter();
	double time = mScene.GetTime();
	tVector pos = character.GetRootPos();

	char buffer[256];
#ifdef _LINUX_
	sprintf(buffer, "Time: %.2fs\nPosition: (%.2f, %.2f, %.2f)\n",
		time, pos[0], pos[1], pos[2]);
#else
	sprintf_s(buffer, "Time: %.2fs\nPosition: (%.2f, %.2f, %.2f)\n",
		time, pos[0], pos[1], pos[2]);
#endif

	std::string str(buffer);

	return str;
}

std::string cDrawScenarioKinMotion::GetName() const
{
	return mScene.GetName();
}

void cDrawScenarioKinMotion::UpdateScene(double time_elapsed)
{
	mScene.Update(time_elapsed);
}

tVector cDrawScenarioKinMotion::GetCamTrackPos() const
{
	return mScene.GetCharPos();
}

tVector cDrawScenarioKinMotion::GetCamStillPos() const
{
	return mScene.GetCharPos();
}

tVector cDrawScenarioKinMotion::GetDefaultCamFocus() const
{
	return gCamFocus0;
}

void cDrawScenarioKinMotion::DrawGround() const
{
	tVector ground_col = GetGroundColor();
	cDrawUtil::SetColor(ground_col);

	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawModeUber3D || draw_mode == eDrawMode3D)
	{
		DrawGround3D();
	}
	else
	{
		DrawGround2D();
	}
}

void cDrawScenarioKinMotion::DrawGround2D() const
{
	const tVector ground_col = tVector(151 / 255.0, 151 / 255.0, 151 / 255.0, 1.0);
	const double marker_spacing = 0.20;
	const double big_marker_spacing = marker_spacing * 5;
	const double marker_h = 0.04;
	const double big_marker_h = 0.075;
	const double ground_h = gGroundHeight;

	tVector focus = mCam.GetFocus();
	double cam_w = mCam.GetWidth();
	double cam_h = mCam.GetHeight();

	double max_x = focus(0) + cam_w;
	double max_y = ground_h;
	double min_x = focus(0) - cam_w;
	double min_y = std::min(focus(1) - cam_h * 0.5f, max_y - 0.05f);

	tVector pos = tVector(focus(0), (min_y + max_y) * 0.5, 0, 0);
	tVector size = tVector(cam_w, (max_y - min_y), 0, 0);

	cDrawUtil::DrawRuler2D(pos, size, ground_col, marker_spacing, big_marker_spacing,
							marker_h, big_marker_h);
}

void cDrawScenarioKinMotion::DrawGround3D() const
{
	const double w = 200;
	const tVector ground_origin = tVector(0, gGroundHeight, 0, 0);
	const tVector tex_size = tVector(0.5, 0.5, 0, 0);

	const cKinCharacter& character = mScene.GetCharacter();
	tVector char_pos = character.GetRootPos();
	char_pos[1] = gGroundHeight;
	tVector a = char_pos - tVector(-0.5 * w, 0, -0.5 * w, 0);
	tVector b = char_pos - tVector(-0.5 * w, 0, 0.5 * w, 0);
	tVector c = char_pos - tVector(0.5 * w, 0, 0.5 * w, 0);
	tVector d = char_pos - tVector(0.5 * w, 0, -0.5 * w, 0);

	tVector min_coord = a - ground_origin;
	tVector max_coord = c - ground_origin;
	min_coord[0] /= tex_size[0];
	min_coord[1] = min_coord[2] / tex_size[1];
	max_coord[0] /= tex_size[0];
	max_coord[1] = max_coord[2] / tex_size[1];

	tVector coord_a = tVector(min_coord[0], min_coord[1], 0, 0);
	tVector coord_b = tVector(min_coord[0], max_coord[1], 0, 0);
	tVector coord_c = tVector(max_coord[0], max_coord[1], 0, 0);
	tVector coord_d = tVector(max_coord[0], min_coord[1], 0, 0);

	cDrawUtil::DrawQuad(a, b, c, d, coord_a, coord_b, coord_c, coord_d);
}


void cDrawScenarioKinMotion::DrawCharacters() const
{
	const cKinCharacter& character = mScene.GetCharacter();
	cDrawCharacter::Draw(character, gLinkWidth, gFilLColor, gLineColor);
}

bool cDrawScenarioKinMotion::EnableAnnotation() const
{
	return (mAnnotateOutputFile != "");
}

void cDrawScenarioKinMotion::Annotate()
{
	if (EnableAnnotation())
	{
		const cKinCharacter& kin_char = mScene.GetCharacter();
		double time = kin_char.GetTime();
		std::string str = std::to_string(time) + "\n";
		cFileUtil::AppendText(str, mAnnotateOutputFile);

		printf("Annotate time: %.5f\n", time);
	}
}
