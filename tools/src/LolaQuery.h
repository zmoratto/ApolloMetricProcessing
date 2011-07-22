#ifndef __VW_LOLA_QUERY_H__
#define __VW_LOLA_QUERY_H__

#include <vw/Cartography.h>

namespace vw {

  class LOLAQuery {
    // These should be hard coded
    std::vector<std::string> m_filenames;
    std::vector<BBox2> m_bboxes;
  public:
    LOLAQuery() {
      std::string base_path("/Users/zmoratto/Data/Moon/LOLA/LOLA_DEM_1024/");
      // Right
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,0),Vector2(30,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,0),Vector2(60,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,0),Vector2(90,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,0),Vector2(120,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,0),Vector2(150,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,0),Vector2(180,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,0),Vector2(210,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,0),Vector2(240,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,0),Vector2(270,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,0),Vector2(300,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,0),Vector2(330,15)));
      m_filenames.push_back(base_path+"LDEM_1024_00N_15N_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,0),Vector2(360,15)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,15),Vector2(30,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,15),Vector2(60,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,15),Vector2(90,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,15),Vector2(120,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,15),Vector2(150,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,15),Vector2(180,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,15),Vector2(210,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,15),Vector2(240,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,15),Vector2(270,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,15),Vector2(300,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,15),Vector2(330,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15N_30N_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,15),Vector2(360,30)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,-15),Vector2(30,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,-15),Vector2(60,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,-15),Vector2(90,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,-15),Vector2(120,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,-15),Vector2(150,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,-15),Vector2(180,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,-15),Vector2(210,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,-15),Vector2(240,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,-15),Vector2(270,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,-15),Vector2(300,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,-15),Vector2(330,0)));
      m_filenames.push_back(base_path+"LDEM_1024_15S_00S_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,-15),Vector2(360,0)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,30),Vector2(30,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,30),Vector2(60,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,30),Vector2(90,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,30),Vector2(120,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,30),Vector2(150,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,30),Vector2(180,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,30),Vector2(210,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,30),Vector2(240,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,30),Vector2(270,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,30),Vector2(300,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,30),Vector2(330,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30N_45N_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,30),Vector2(360,45)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,-30),Vector2(30,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,-30),Vector2(60,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,-30),Vector2(90,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,-30),Vector2(120,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,-30),Vector2(150,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,-30),Vector2(180,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,-30),Vector2(210,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,-30),Vector2(240,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,-30),Vector2(270,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,-30),Vector2(300,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,-30),Vector2(330,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_30S_15S_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,-30),Vector2(360,-15)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,45),Vector2(30,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,45),Vector2(60,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,45),Vector2(90,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,45),Vector2(120,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,45),Vector2(150,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,45),Vector2(180,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,45),Vector2(210,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,45),Vector2(240,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,45),Vector2(270,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,45),Vector2(300,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,45),Vector2(330,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45N_60N_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,45),Vector2(360,60)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,-45),Vector2(30,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,-45),Vector2(60,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,-45),Vector2(90,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,-45),Vector2(120,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,-45),Vector2(150,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,-45),Vector2(180,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,-45),Vector2(210,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,-45),Vector2(240,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,-45),Vector2(270,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,-45),Vector2(300,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,-45),Vector2(330,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_45S_30S_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,-45),Vector2(360,-30)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,60),Vector2(30,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,60),Vector2(60,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,60),Vector2(90,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,60),Vector2(120,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,60),Vector2(150,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,60),Vector2(180,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,60),Vector2(210,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,60),Vector2(240,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,60),Vector2(270,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,60),Vector2(300,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,60),Vector2(330,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60N_75N_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,60),Vector2(360,75)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,-60),Vector2(30,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,-60),Vector2(60,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,-60),Vector2(90,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,-60),Vector2(120,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,-60),Vector2(150,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,-60),Vector2(180,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,-60),Vector2(210,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,-60),Vector2(240,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,-60),Vector2(270,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,-60),Vector2(300,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,-60),Vector2(330,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_60S_45S_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,-60),Vector2(360,-45)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,75),Vector2(30,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,75),Vector2(60,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,75),Vector2(90,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,75),Vector2(120,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,75),Vector2(150,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,75),Vector2(180,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,75),Vector2(210,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,75),Vector2(240,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,75),Vector2(270,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,75),Vector2(300,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,75),Vector2(330,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75N_90N_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,75),Vector2(360,90)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,-75),Vector2(30,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,-75),Vector2(60,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,-75),Vector2(90,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,-75),Vector2(120,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,-75),Vector2(150,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,-75),Vector2(180,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,-75),Vector2(210,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,-75),Vector2(240,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,-75),Vector2(270,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,-75),Vector2(300,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,-75),Vector2(330,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_75S_60S_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,-75),Vector2(360,-60)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_000_030.tif");
      m_bboxes.push_back(BBox2(Vector2(0,-90),Vector2(30,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_030_060.tif");
      m_bboxes.push_back(BBox2(Vector2(30,-90),Vector2(60,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_060_090.tif");
      m_bboxes.push_back(BBox2(Vector2(60,-90),Vector2(90,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_090_120.tif");
      m_bboxes.push_back(BBox2(Vector2(90,-90),Vector2(120,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_120_150.tif");
      m_bboxes.push_back(BBox2(Vector2(120,-90),Vector2(150,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_150_180.tif");
      m_bboxes.push_back(BBox2(Vector2(150,-90),Vector2(180,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_180_210.tif");
      m_bboxes.push_back(BBox2(Vector2(180,-90),Vector2(210,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_210_240.tif");
      m_bboxes.push_back(BBox2(Vector2(210,-90),Vector2(240,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_240_270.tif");
      m_bboxes.push_back(BBox2(Vector2(240,-90),Vector2(270,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_270_300.tif");
      m_bboxes.push_back(BBox2(Vector2(270,-90),Vector2(300,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_300_330.tif");
      m_bboxes.push_back(BBox2(Vector2(300,-90),Vector2(330,-75)));
      m_filenames.push_back(base_path+"LDEM_1024_90S_75S_330_360.tif");
      m_bboxes.push_back(BBox2(Vector2(330,-90),Vector2(360,-75)));

      VW_ASSERT( m_filenames.size() == 144, LogicErr() << "Programmer should provide 144 inputs for entire coverage of Moon" );
      VW_ASSERT( m_bboxes.size() == 144, LogicErr() << "Programmer should provide 144 inputs for entire coverage of Moon" );
    }

    void print() {
      for ( size_t i = 0; i < m_bboxes.size(); i++ ) {
        std::cout << "File: " << m_filenames[i] << "\n";
        std::cout << "BBox: " << m_bboxes[i] << "\n";
      }
    }

    std::pair<cartography::GeoReference, std::string>
    find_tile( Vector2 lonlat ) {
      if ( lonlat[0] < 0 )
        lonlat[0] += 360;
      for ( size_t i = 0; i < m_bboxes.size(); i++ ) {
        if ( m_bboxes[i].contains( lonlat ) ) {
          std::pair<cartography::GeoReference, std::string> result;
          cartography::read_georeference( result.first,
                                          m_filenames[i] );
          result.second = m_filenames[i];
          return result;
        }
      }
      vw_throw( ArgumentErr() << "Unable to find match?" );
    }

    std::pair<cartography::GeoReference, std::string>
    find_tile( Vector3 llr ) {
      return find_tile( Vector2(subvector( llr, 0, 2 )) );
    }
  };


}

#endif//__VW_LOLA_QUERY_H__
