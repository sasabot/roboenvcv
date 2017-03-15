#include "roboenvcv/roboenvcv_extra.hh"

//////////////////////////////////////////////////
void roboenvcv::PatchBoundsForOcr
(roboenvcv::objectarea &_it, cv::Mat &_img, float _margin_top,
 float _margin_right, float _margin_bottom, float _margin_left)
{
  int margin_left = _margin_left;
  _it.bounds2d.x -= margin_left;
  if (_it.bounds2d.x < 0) {
    _it.bounds2d.x = 0;
    margin_left = _it.bounds2d.x;
  }

  _it.bounds2d.width += margin_left + _margin_right;
  if (_it.bounds2d.x + _it.bounds2d.width >= _img.cols)
    _it.bounds2d.width = _img.cols - _it.bounds2d.x;

  int margin_top = _margin_top;
  _it.bounds2d.y -= margin_top;
  if (_it.bounds2d.y < 0) {
    _it.bounds2d.y = 0;
    margin_top = _it.bounds2d.y;
  }

  _it.bounds2d.height += margin_top + _margin_bottom;
  if (_it.bounds2d.y + _it.bounds2d.height >= _img.rows)
    _it.bounds2d.height = _img.rows - _it.bounds2d.y;
}

//////////////////////////////////////////////////
std::vector<int> roboenvcv::FindTargetWithOcr
(std::vector<std::string> _target_name, std::vector<roboenvcv::objectarea> &_scene,
 cv::Mat &_img, windows::interface::WindowsInterfacePtr _windows,
 std::string _debug_folder)
{
  std::vector<cv::Mat> images(_scene.size());
  auto img = images.begin();
  for (auto obj = _scene.begin(); obj != _scene.end(); ++obj) {
    *img = _img(obj->bounds2d);
    ++img;
  }

  // with images, get OCR result
  auto ocr = _windows->OCR(images);

  // save ocr results to scene
  for (unsigned int i = 0; i < _scene.size(); ++i) {
    auto res = ocr.begin() + i;
    auto obj = _scene.begin() + i;
    for (auto txt = res->begin(); txt != res->end(); ++txt)
      if (*txt != "") // name may already have a result so push back
        obj->properties.name.push_back(*txt);
  }

  // find best matches
  std::vector<std::pair<int, int> > candidates;
  // longer match is better match, we want to find best match first
  std::sort(_target_name.begin(), _target_name.end(),
            [](std::string a, std::string b){return (a.length() > b.length());});
  for (unsigned int i = 0; i < _scene.size(); ++i) {
    auto obj = _scene.begin() + i;
    if (!obj->visible3d) continue; // for now, ignore non-visible objects
    auto res = ocr.begin() + i;
    bool go_to_next = false;
    for (auto str = _target_name.begin(); str != _target_name.end(); ++str) {
      // OCR result is first name English, second name Japanese
      std::string word = res->at(0);
      // if byte is multi byte, use second name
      if ((0x80 & (*str)[0]) != 0) word = res->at(1);
      if (word == "") { // object has no result, go to next object
        go_to_next = true;
        break;
      } else if (str->find(word) != std::string::npos ||
                 word.find(*str) != std::string::npos) {
        candidates.push_back({i, static_cast<int>(str - _target_name.begin())});
        go_to_next = true;
        break; // found best result, go to next object 
      }
    }
    if (!go_to_next) // this means some words were at least found
      candidates.push_back({i, _target_name.size()});
  }

  if (candidates.size() == 0) // if no candidates, return
    return std::vector<int> {};

  // order candidates with best match
  std::sort(candidates.begin(), candidates.end(),
            [](std::pair<int, float> x, std::pair<int, float> y){
              return (x.second < y.second);
            });

  // divide candidates into field and add to result
  std::vector<std::vector<std::pair<int, float>> > ordered_candidates(1);
  auto oc = ordered_candidates.begin();
  float best_score = candidates.begin()->second;
  for (auto obj = candidates.begin(); obj != candidates.end(); ++obj) {
    auto s = _scene.begin() + obj->first;
    if (obj->second == best_score) {
      oc->push_back({obj->first, s->center3d.norm()});
    } else { // if not same score, add to next field
      best_score = obj->second;
      ordered_candidates.push_back({{obj->first, s->center3d.norm()}});
      oc = ordered_candidates.end() - 1;
    }
  }

  // sort matches by distance and add to result
  std::vector<int> result(candidates.size());
  auto it = result.begin();
  for (auto c = ordered_candidates.begin(); c != ordered_candidates.end(); ++c) {
    std::sort(c->begin(), c->end(),
              [](std::pair<int, float> x, std::pair<int, float> y){
                return (x.second < y.second);
              });
    for (auto obj = c->begin(); obj != c->end(); ++obj)
      *it++ = obj->first;
  }

  return result;
}
