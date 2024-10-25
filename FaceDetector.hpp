#pragma once

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>

#include "Image.hpp"
#include "Draw.hpp"
#include "ImageIO.hpp"
#include "FunctionTimer.hpp"

class FaceDetector
{
public: 

enum class FaceKeyPoint : int
{
  LEFT_EYE = 0,
  RIGHT_EYE = 1,
  NOSE_TIP = 2,
  MOUTH_CENTER = 3,
  RIGHT_EAR_TRAGION = 4,
  LEFT_EAR_TRAGION = 5
};

class Detection
{
public:
  float score;
  float class_id;
  float xmin;
  float ymin;
  float width;
  float height;

  float rightEyeX;
  float rightEyeY;
  float leftEyeX;
  float leftEyeY;
  float noseTipX;
  float noseTipY;
  float mouthX;
  float mouthY;
  float rightEarX;
  float rightEarY;
  float leftEarX;
  float leftEarY;

  Detection()
  {
    this->score = 0.0;
    this->class_id = 0.0;
    this->xmin = 0.0;
    this->ymin = 0.0;
    this->width = 0.0;
    this->height = 0.0;
  }

  Detection(float score, float class_id, float xmin, float ymin, float width, float height)
  {
    this->score = score;
    this->class_id = class_id;
    this->xmin = xmin;
    this->ymin = ymin;
    this->width = width;
    this->height = height;
  }

  // Scale the results from normalized coords to the size of some destination image
  void scaleResults(float xScale, float yScale)
  {
    xmin *= xScale;
    width *= xScale;
    rightEyeX *= xScale;
    leftEyeX *= xScale;
    noseTipX *= xScale;
    mouthX *= xScale;
    rightEarX *= xScale;
    leftEarX *= xScale;

    ymin *= yScale;
    height *= yScale;
    rightEyeY *= yScale;
    leftEyeY *= yScale;
    noseTipY *= yScale;
    mouthY *= yScale;
    rightEarY *= yScale;
    leftEarY *= yScale;
  }

  float centerX()
  {
    return xmin + width / 2.0f;
  }

  float centerY()
  {
    return ymin + height / 2.0f;
  }
};

private:
/*
 * Sort vector based on the given indices.
 */ 
static std::vector<float> lgt_sort_vec(std::vector<float> const &a, std::vector<int> indices)
{
  PROFILE_FUNCTION;
  
  std::vector<float> retVec;

  for (auto i : indices)
  {
      retVec.push_back(a.at(i));
  }
  return retVec;
}

/*
 * Vector Sclicer.
 * example: 
 * v = [0, 1, 2, 3, 4, 5]
 * slice(v, 0, 3) --> 0, 1, 2.
 * slice(v, 2, 5) --> 2, 3, 4.
 * slice(v, 0, v.size()) --> 0, 1, 2, 3, 4, 5.
 * slice(v, 0, v.size()-1) --> 0, 1, 2, 3, 4.
 */
template<typename T>
static std::vector<T> lgt_slice(std::vector<T> const &v, int m, int n)
{
  PROFILE_FUNCTION;

  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n;

  std::vector<T> vec(first, last);
  return vec;
}

/*
 * Create vector of max values.
 */ 
static std::vector<float> lgt_vec_maximum(std::vector<float> const &a, std::vector<int> indices)
{
  PROFILE_FUNCTION;

  float compare = 0.0;
  std::vector<float> retVec;

  if (!indices.empty())
  {
      compare = a.at(indices.back());
      std::vector<int> indicesSlice = lgt_slice(indices, 0, indices.size()-1);
      retVec = lgt_sort_vec(a, indicesSlice);
  }
  else
  {
      retVec = a;
  }

  for(int i = 0; i < retVec.size(); i++)
  {
      retVec.at(i) = retVec.at(i) > compare ? retVec.at(i) : compare;
  }
  return retVec;
}


/*
 * Create vector of min values, comparing with max values.
 */ 
static std::vector<float> lgt_vec_minimum(std::vector<float> const &a, std::vector<int> indices)
{
  PROFILE_FUNCTION;

  float compare = 0.0;
  std::vector<float> retVec;

  if (!indices.empty())
  {
      compare = a.at(indices.back());
      std::vector<int> indicesSlice = lgt_slice(indices, 0, indices.size()-1);
      retVec = lgt_sort_vec(a, indicesSlice);
  }
  else
  {
      retVec = a;
  }

  for(int i = 0; i < retVec.size(); i++)
  {
      retVec.at(i) = retVec.at(i) < compare ? retVec.at(i) : compare;
  }
  return retVec;
}


/*
 * Intersection Over Union (IoU).
 */ 
static std::vector<float> lgt_iou(std::vector<float> const &a, std::vector<float> const &inter, std::vector<int> indices)
{
  PROFILE_FUNCTION;

  float largest = a.at(indices.back());
  indices.pop_back();
  std::vector<float> retVec = lgt_sort_vec(a, indices);
  std::vector<float> totalArea(retVec.size());

  std::transform(retVec.begin(), retVec.end(), totalArea.begin(), bind2nd(std::plus<float>(), largest));
  std::vector<float> iou(inter.size());
  std::transform(totalArea.begin(), totalArea.end(), inter.begin(), iou.begin(), std::minus<float>());
  std::transform(inter.begin(), inter.end(), iou.begin(), iou.begin(), std::divides<float>());

  return iou;
}

/*
 * IOU Argsort logic.
 */ 
static std::vector<int> lgt_iou_argsort(std::vector<float> const &a, float threshold)
{
  PROFILE_FUNCTION;

  std::vector<int> vArg;
  uint32_t index = 0;

  for (auto i : a)
  {
      if (a.at(i) <= threshold)
      {
          vArg.push_back(index);
      }
      index++;
  }

  return vArg;
}


/*
 * Generic Argsort
 * Sort based on order (largest or smallest) and return the indexes.
 */ 
template <typename Iter, typename Compare>
static std::vector<int> argsort(Iter begin, Iter end, Compare comp)
{
  PROFILE_FUNCTION;

	// Begin Iterator, End Iterator, Comp
	std::vector<std::pair<int, Iter> > pairList; // Pair Vector
	std::vector<int> ret;                        // Will hold the indices

	int i = 0;
	for (auto it = begin; it < end; it++)
	{
		std::pair<int, Iter> pair(i, it); // 0: Element1, 1:Element2...
		pairList.push_back(pair);         // Add to list
		i++;
	}
	// Stable sort the pair vector
	std::stable_sort(pairList.begin(), pairList.end(),
		[comp](std::pair<int, Iter> prev, std::pair<int, Iter> next) -> bool
	{return comp(*prev.second, *next.second); }  // This is the important part explained below
	);

	for (auto i : pairList)
		ret.push_back(i.first);

	return ret;
}

struct SsdAnchorsCalculatorOptions
{
  // Size of input images.
  uint16_t input_size_width = 128;
  uint16_t input_size_height = 128;

  // Min and max scales for generating anchor boxes on feature maps.
  float min_scale = 0.1484375f;
  float max_scale = 0.75f;

  // The offset for the center of anchors. The value is in the scale of stride.
  // E.g. 0.5 meaning 0.5 * |current_stride| in pixels.
  float anchor_offset_x = 0.5f;
  float anchor_offset_y = 0.5f;

  // An additional anchor is added with this aspect ratio and a scale
  // interpolated between the scale for a layer and the scale for the next layer
  // (1.0 for the last layer). This anchor is not included if this value is 0.
  float interpolated_scale_aspect_ratio = 1.0f;

  // A boolean to indicate whether the fixed 3 boxes per location is used in the lowest layer.
  bool reduce_boxes_in_lowest_layer = false; 

  // Whether use fixed width and height (e.g. both 1.0f) for each anchor.
  // This option can be used when the predicted anchor width and height are in  pixels.
  bool fixed_anchor_size = false;

  // Strides of each output feature maps.
  std::vector<int> strides = {8, 16, 16, 16};

  // Number of output feature maps to generate the anchors on.
  uint8_t num_layers = 4;

  // List of different aspect ratio to generate anchors.
  std::vector<float> aspect_ratios = {1.0};
};


class Anchor
{
    public:
        float x_center;
        float y_center;
        float h;
        float w;

        Anchor()
        {
            this->x_center = 0;
            this->y_center = 0;
            this->h = 0;
            this->w = 0;
        }

        std::string to_str()
        {
            std::string retstr;

            retstr += "x_center : " + std::to_string(this->x_center) + "\n";
            retstr += "y_center : " + std::to_string(this->y_center) + "\n";
            retstr += "h : " + std::to_string(this->h) + "\n";
            retstr += "w : " + std::to_string(this->w) + "\n";
            return retstr;
        }

        ~Anchor() {};
};

struct TfLiteTensorsToDetectionsCalculatorOptions
{
  uint32_t num_classes = 1;
  uint32_t num_boxes = 896;
  uint32_t num_coords = 16; // 1 * 4 face box (x1,y1,x2,y2), + 6 * 2 key points = 16
  uint32_t keypoint_coord_offset = 4; // 1 * 4 face box
  uint32_t num_keypoints = 6; // eyeL, eyeR, nose, mouth, earL, earR
  uint32_t num_values_per_keypoint = 2; // x and y
  uint32_t box_coord_offset; // box is first item
  float x_scale = 0.0f; // set these to input image size
  float y_scale = 0.0f;
  float w_scale = 0.0f;
  float h_scale = 0.0f;
  float score_clipping_thresh = 100.0f;
  float min_score_thresh = 0.75f;
  bool apply_exponential_on_box_size = false;
  bool reverse_output_order = false;
  bool sigmoid_score = false;
  bool flip_vertically = false;
};


static std::vector<float> lgt_decode_box(std::vector<float> raw_boxes, std::vector<Anchor> anchors, const TfLiteTensorsToDetectionsCalculatorOptions& options, uint32_t idx)
{
  PROFILE_FUNCTION;

  std::vector<float> box_data(options.num_coords, 0.0);
      
  uint32_t box_offset = idx * options.num_coords + options.box_coord_offset;

  float y_center = raw_boxes[box_offset];
  float x_center = raw_boxes[box_offset + 1];
  float h = raw_boxes[box_offset + 2];
  float w = raw_boxes[box_offset + 3];

  if (options.reverse_output_order)
  {
      x_center = raw_boxes[box_offset];
      y_center = raw_boxes[box_offset + 1];
      w = raw_boxes[box_offset + 2];
      h = raw_boxes[box_offset + 3];
  }

  x_center = x_center / options.x_scale * anchors[idx].w + anchors[idx].x_center;
  y_center = y_center / options.y_scale * anchors[idx].h + anchors[idx].y_center;

  if (options.apply_exponential_on_box_size)
  {
      h = exp(h / options.h_scale) * anchors[idx].h;
      w = exp(w / options.w_scale) * anchors[idx].w;
  }
  else
  {
      h = h / options.h_scale * anchors[idx].h;
      w = w / options.w_scale * anchors[idx].w;
  }

  float ymin = y_center - h / 2.0;
  float xmin = x_center - w / 2.0;
  float ymax = y_center + h / 2.0;
  float xmax = x_center + w / 2.0;

  box_data[0] = ymin;
  box_data[1] = xmin;
  box_data[2] = ymax;
  box_data[3] = xmax;

  if (options.num_keypoints)
  {
      for (uint32_t k = 0; k < options.num_keypoints; k++)
      {
          uint32_t offset = idx * options.num_coords + options.keypoint_coord_offset + k * options.num_values_per_keypoint;

          float keypoint_y = raw_boxes[offset];
          float keypoint_x = raw_boxes[offset + 1];
          if (options.reverse_output_order)
          {
              keypoint_x = raw_boxes[offset];
              keypoint_y = raw_boxes[offset + 1];
          }

          box_data[4 + k * options.num_values_per_keypoint] = keypoint_x / options.x_scale * anchors[idx].w + anchors[idx].x_center;
          box_data[4 + k * options.num_values_per_keypoint  + 1] = keypoint_y / options.y_scale * anchors[idx].h + anchors[idx].y_center;
      }
  }

  return box_data;
}


static Detection lgt_convert_to_detection(float box_ymin, float box_xmin, float box_ymax, float box_xmax, float score, float class_id, bool flip_vertically) 
{
  PROFILE_FUNCTION;
  Detection detection = Detection(score, class_id, box_xmin, (flip_vertically ? 1.0 - box_ymax : box_ymin), (box_xmax - box_xmin), (box_ymax - box_ymin));
  return detection;
}


static std::vector<Detection> lgt_convert_to_detections(std::vector<float> raw_boxes, std::vector<Anchor> anchors, 
                          std::vector<float> detection_scores, std::vector<float> detection_classes, 
                          const TfLiteTensorsToDetectionsCalculatorOptions& options)
{
  PROFILE_FUNCTION;

  std::vector<Detection> output_detections;

  for (int i = 0; i < options.num_boxes; i++)
  {
      if (detection_scores[i] < options.min_score_thresh)
      {
          continue;
      }
      
      std::vector<float> box_data = lgt_decode_box(raw_boxes, anchors, options, i);
      Detection detection = lgt_convert_to_detection(
              box_data[0], box_data[1],
              box_data[2], box_data[3],
              detection_scores[i], detection_classes[i], options.flip_vertically);
      
      detection.leftEyeX = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::LEFT_EYE * options.num_values_per_keypoint];
      detection.leftEyeY = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::LEFT_EYE * options.num_values_per_keypoint + 1];

      detection.rightEyeX = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::RIGHT_EYE * options.num_values_per_keypoint];
      detection.rightEyeY = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::RIGHT_EYE * options.num_values_per_keypoint + 1];

      detection.noseTipX = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::NOSE_TIP * options.num_values_per_keypoint];
      detection.noseTipY = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::NOSE_TIP * options.num_values_per_keypoint + 1];

      detection.mouthX = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::MOUTH_CENTER * options.num_values_per_keypoint];
      detection.mouthY = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::MOUTH_CENTER * options.num_values_per_keypoint + 1];

      detection.rightEarX = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::RIGHT_EAR_TRAGION * options.num_values_per_keypoint];
      detection.rightEarY = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::RIGHT_EAR_TRAGION * options.num_values_per_keypoint + 1];

      detection.leftEarX = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::LEFT_EAR_TRAGION * options.num_values_per_keypoint];
      detection.leftEarY = box_data[options.keypoint_coord_offset + (int)FaceKeyPoint::LEFT_EAR_TRAGION * options.num_values_per_keypoint + 1];

      output_detections.push_back(detection);
  }
  return output_detections;
}


/*
 * Postprocessing on CPU for model without postprocessing op. E.g. output
 * raw score tensor and box tensor. Anchor decoding will be handled below.
 */ 
static std::vector<Detection> lgt_process_cpu(std::vector<float> raw_boxes, std::vector<float> raw_scores, std::vector<Anchor> anchors, const TfLiteTensorsToDetectionsCalculatorOptions& options)
{
  PROFILE_FUNCTION;

  std::vector<float> detection_scores(options.num_boxes, 0.0);
  std::vector<float> detection_classes(options.num_boxes, 0.0);
  std::vector<Detection> output_detections;

  // Filter classes by scores.
  for (int i = 0; i < options.num_boxes; i++)
  {
      int class_id = -1;
      float max_score = std::numeric_limits<float>::min();

      // Find the top score for box i.
      for (int score_idx = 0; score_idx < options.num_classes; score_idx++)
      {
          float score = raw_scores[i * options.num_classes + score_idx];
          if (options.sigmoid_score)
          {
              if (options.score_clipping_thresh > 0)
              {
                  score = score < -options.score_clipping_thresh ? -options.score_clipping_thresh : score;
                  score = score > options.score_clipping_thresh ? options.score_clipping_thresh : score;
              }
              score = 1.0 / (1.0 + exp(-score));
          }
          if (max_score < score)
          {
              max_score = score;
              class_id = score_idx;
          }
      }
      detection_scores[i] = max_score;
      detection_classes[i] = class_id;
  }

  output_detections = lgt_convert_to_detections(raw_boxes, anchors, detection_scores, 
                                                detection_classes, options);
  return output_detections;
}


static std::vector<Detection> lgt_orig_nms(std::vector<Detection> detections, float threshold)
{
  PROFILE_FUNCTION;

  if (detections.size() <= 0)
  {
      return std::vector<Detection>();
  }

  std::vector<float> x1;
  std::vector<float> x2;
  std::vector<float> y1;
  std::vector<float> y2;
  std::vector<float> s;

  for (std::vector<Detection>::iterator ptr = detections.begin(); ptr < detections.end(); ptr++)
  {
      x1.push_back(ptr->xmin);
      x2.push_back(ptr->xmin + ptr->width);
      y1.push_back(ptr->ymin);
      y2.push_back(ptr->ymin + ptr->height);
      s.push_back(ptr->score);
  }
  
  std::vector<float> X(x1.size());
  std::vector<float> Y(y1.size());
  std::vector<float> area(x1.size());

  std::transform(x2.begin(), x2.end(), x1.begin(), X.begin(), std::minus<float>());
  std::transform(X.begin(), X.end(), X.begin(), bind2nd(std::plus<float>(), 1));

  std::transform(y2.begin(), y2.end(), y1.begin(), Y.begin(), std::minus<float>());
  std::transform(Y.begin(), Y.end(), Y.begin(), bind2nd(std::plus<float>(), 1));

  std::transform(X.begin(), X.end(), Y.begin(), area.begin(), std::multiplies<float>());
    
  std::vector<int> indices = argsort(s.begin(), s.end(), std::less<float>());

  std::vector<float> pick;
  while (indices.size() > 0)
  {
      std::vector<float> xx1 = lgt_vec_maximum(x1, indices);   
      std::vector<float> yy1 = lgt_vec_maximum(y1, indices);   
      std::vector<float> xx2 = lgt_vec_minimum(x2, indices);   
      std::vector<float> yy2 = lgt_vec_minimum(y2, indices);   

      std::vector<float> XX(xx1.size());
      std::vector<float> YY(yy1.size());

      std::transform(xx2.begin(), xx2.end(), xx1.begin(), XX.begin(), std::minus<float>());
      std::transform(XX.begin(), XX.end(), XX.begin(), bind2nd(std::plus<float>(), 1.0));

      std::transform(yy2.begin(), yy2.end(), yy1.begin(), YY.begin(), std::minus<float>());
      std::transform(YY.begin(), YY.end(), YY.begin(), bind2nd(std::plus<float>(), 1.0));

      std::vector<float> w = lgt_vec_maximum(XX, std::vector<int>());
      std::vector<float> h = lgt_vec_maximum(YY, std::vector<int>());

      std::vector<float> inter(w.size());
      std::transform(w.begin(), w.end(), h.begin(), inter.begin(), std::multiplies<float>());

      std::vector<float> iou = lgt_iou(area, inter, indices);
      pick.push_back(indices.back());

      indices = lgt_iou_argsort(iou, threshold);
  }

  std::vector<Detection> retDetections;

  for (int i = 0; i < pick.size(); i++)
  {
      retDetections.push_back(detections.at(pick[i]));
  }
  return retDetections;
}

static std::vector<Anchor> lgt_gen_anchors(const SsdAnchorsCalculatorOptions& options)
{
  PROFILE_FUNCTION;

  std::vector<Anchor> anchors;

  // Verify the options.
  if (options.strides.size() != options.num_layers)
  {
    throw std::runtime_error("strides_size and num_layers must be equal.");
  }

  uint32_t layer_id = 0;
  while (layer_id < options.strides.size())
  {
      std::vector<float> anchor_height;
      std::vector<float> anchor_width;
      std::vector<float> aspect_ratios;
      std::vector<float> scales;

      // For same strides, we merge the anchors in the same order.
      uint32_t last_same_stride_layer = layer_id;
      while (last_same_stride_layer < options.strides.size() && options.strides[last_same_stride_layer] == options.strides[layer_id])
      {
          float scale = options.min_scale + (options.max_scale - options.min_scale) * 1.0 * last_same_stride_layer / (options.strides.size() - 1.0);
          if (last_same_stride_layer == 0 && options.reduce_boxes_in_lowest_layer)
          {
              // For first layer, it can be specified to use predefined anchors.
              aspect_ratios.push_back(1.0);
              aspect_ratios.push_back(2.0);
              aspect_ratios.push_back(0.5);
              scales.push_back(0.1);
              scales.push_back(scale);
              scales.push_back(scale);
          }
          else
          {
              for (size_t aspect_ratio_id = 0; aspect_ratio_id < options.aspect_ratios.size(); aspect_ratio_id++)
              {
                  aspect_ratios.push_back(options.aspect_ratios[aspect_ratio_id]);
                  scales.push_back(scale);
              }

              if (options.interpolated_scale_aspect_ratio > 0.0)
              {
                  float scale_next = (last_same_stride_layer == options.strides.size() - 1) ? 1.0 : (options.min_scale + (options.max_scale - options.min_scale) * 1.0 * (last_same_stride_layer + 1) / (options.strides.size() - 1.0));
                  scales.push_back(sqrt(scale * scale_next));
                  aspect_ratios.push_back(options.interpolated_scale_aspect_ratio);
              }
          }
          last_same_stride_layer += 1;
      }
      
      for (size_t i = 0; i < aspect_ratios.size(); i++)
      {
          float ratio_sqrts = sqrt(aspect_ratios[i]);
          anchor_height.push_back(scales[i] / ratio_sqrts);
          anchor_width.push_back(scales[i] * ratio_sqrts);
      }

      uint32_t stride = options.strides[layer_id];
      uint32_t feature_map_height = ceil(1.0 * options.input_size_height / stride);
      uint32_t feature_map_width = ceil(1.0 * options.input_size_width / stride);

      for (size_t y = 0; y < feature_map_height; y++)
      {
          for (size_t x = 0; x < feature_map_width; x++)
          {
              for (uint32_t anchor_id = 0; anchor_id < anchor_height.size(); anchor_id++)
              {
                  float x_center = (x + options.anchor_offset_x) * 1.0 / feature_map_width;
                  float y_center = (y + options.anchor_offset_y) * 1.0 / feature_map_height;
                  float w = 0;
                  float h = 0;
                  if (options.fixed_anchor_size)
                  {
                      w = 1.0;
                      h = 1.0;
                  }
                  else
                  {
                      w = anchor_width[anchor_id];
                      h = anchor_height[anchor_id];
                  }
                  Anchor new_anchor;
                  new_anchor.x_center = x_center;
                  new_anchor.y_center = y_center;
                  new_anchor.h = h;
                  new_anchor.w = w;
                  anchors.push_back(new_anchor);
              }
          }
      }
      layer_id = last_same_stride_layer;
  }
  return anchors;
}

  template <typename Color> 
  void loadImage(const Image<Color>& image)
  {
    PROFILE_FUNCTION;

    if (image.width() != (size_t)modelConfig_.options.input_size_width && 
        image.height() != (size_t)modelConfig_.options.input_size_height)
    {
      throw std::runtime_error("Input image is wrong size!");
    }

    // Copy the scaled image into input tensor, converting to its weird floating point format
    int input = interpreter_->inputs()[0];
    TfLiteTensor* input_tensor = interpreter_->tensor(input);
    RGB3f<-1.0f, 1.0f>* dst = (RGB3f<-1.0f, 1.0f>*)input_tensor->data.f;
    const Color* src = image.pixel();
    size_t size = (size_t)modelConfig_.options.input_size_width * (size_t)modelConfig_.options.input_size_height;
    for (size_t i = 0; i < size; ++i)
    {
      dst[i] = src[i];
    }
  }

public:
  struct SSDOptions
  {
    int num_layers;
    int input_size_height;
    int input_size_width;
    float anchor_offset_x;
    float anchor_offset_y;
    std::vector<int> strides;
    float interpolated_scale_aspect_ratio;
  };

  struct ModelConfig
  {
    const char* name;
    const SSDOptions options;
  };

  // 128x128 image, assumed to be mirrored
  const static ModelConfig FrontModel;

  // 256x256 image, not mirrored
  const static ModelConfig BackModel;

  // 128x128 image, assumed to be mirrored; best for short range images
  //         (i.e. faces within 2 metres from the camera)
  const static ModelConfig ShortModel;

  // 192x192 image, assumed to be mirrored; dense; best for mid-ranges
  //        (i.e. faces within 5 metres from the camera)
  const static ModelConfig FullModel;

  // 192x192 image, assumed to be mirrored; sparse; best for
  //         mid-ranges (i.e. faces within 5 metres from the camera)
  //         this model is up ~30% faster than `FULL` when run on the CPU
  const static ModelConfig FullSparseModel;

  FaceDetector(const ModelConfig& modelConfig) 
    : modelConfig_(modelConfig) 
    , ssd_anchors_calculator_options {
        input_size_width: (uint16_t) modelConfig.options.input_size_width,
        input_size_height: (uint16_t) modelConfig.options.input_size_height,
        min_scale : 0.1484375f,
        max_scale : 0.75f,
        anchor_offset_x: modelConfig.options.anchor_offset_x,
        anchor_offset_y: modelConfig.options.anchor_offset_y,
        interpolated_scale_aspect_ratio : modelConfig.options.interpolated_scale_aspect_ratio,
        reduce_boxes_in_lowest_layer : false,
        fixed_anchor_size : true,
        strides: modelConfig.options.strides,
        num_layers: (uint8_t)modelConfig.options.num_layers 
    }
    , options { 
        num_classes: 1,
        num_boxes: 896,
        num_coords: 16, 
        keypoint_coord_offset: 4,
        num_keypoints: 6, 
        num_values_per_keypoint: 2, 
        box_coord_offset: 0, 
        x_scale: (float)modelConfig.options.input_size_width, 
        y_scale: (float)modelConfig.options.input_size_height, 
        w_scale: (float)modelConfig.options.input_size_width, 
        h_scale: (float)modelConfig.options.input_size_height, 
        score_clipping_thresh: 100.0, 
        min_score_thresh: 0.75, 
        apply_exponential_on_box_size: false, 
        reverse_output_order: true,
        sigmoid_score: true,
        flip_vertically: false 
    }
    , anchors(lgt_gen_anchors(ssd_anchors_calculator_options))
  {
    // Load the model
    model_ = tflite::FlatBufferModel::BuildFromFile(modelConfig_.name);
    if (model_ == nullptr) throw std::runtime_error("Unable to load model!");

    // Build the interpreter
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model_, resolver);
    builder(&interpreter_);
    if (interpreter_ == nullptr) throw std::runtime_error("Unable to build interpreter!");

    // Allocate tensor buffers
    if (interpreter_->AllocateTensors() != kTfLiteOk) throw std::runtime_error("Unable to allocate tensors!");
  }

  template <typename Color> 
  std::vector<Detection> detect(const Image<Color>& image)
  {
    PROFILE_FUNCTION;
    
    loadImage(image);

    // Run Inference
    {
      PROFILE(inference);
      if (interpreter_->Invoke() != kTfLiteOk) throw std::runtime_error("Invoke interpreter failed!");
    }

    // Read output buffers
    // 1. Regressor
    int reg_output = interpreter_->outputs()[0];
    TfLiteTensor* reg_output_tensor = interpreter_->tensor(reg_output);
    TfLiteIntArray* reg_output_dims = interpreter_->tensor(reg_output)->dims;
    int reg_rows = reg_output_dims->data[reg_output_dims->size - 2];
    int reg_cols = reg_output_dims->data[reg_output_dims->size - 1];
    float *regressors = reg_output_tensor->data.f;
    std::vector<float> regressorVec(reg_rows * reg_cols);
    memcpy(&(regressorVec[0]), regressors, sizeof(float) * reg_rows * reg_cols);

    // 2. Classifier 
    int cls_output = interpreter_->outputs()[1];
    TfLiteTensor* cls_output_tensor = interpreter_->tensor(cls_output);
    TfLiteIntArray* cls_output_dims = interpreter_->tensor(cls_output)->dims;
    int cls_rows = cls_output_dims->data[cls_output_dims->size - 2];
    int cls_cols = cls_output_dims->data[cls_output_dims->size - 1];
    float *classifiers = cls_output_tensor->data.f;
    std::vector<float> classifiersVec(cls_rows * cls_cols);
    memcpy(&(classifiersVec[0]), classifiers, sizeof(float) * cls_rows * cls_cols);

    std::vector<Detection> detections = lgt_process_cpu(regressorVec, classifiersVec, anchors, options);
    //detections = lgt_orig_nms(detections, 0.85);
    detections = lgt_orig_nms(detections, 0.30);

    return detections;
  }

  size_t inputImageWidth() const
  {
    return modelConfig_.options.input_size_width;
  }
  
  size_t inputImageHeight() const
  {
    return modelConfig_.options.input_size_height;
  }

private:
  ModelConfig modelConfig_;
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  SsdAnchorsCalculatorOptions ssd_anchors_calculator_options;
  TfLiteTensorsToDetectionsCalculatorOptions options;
  std::vector<Anchor> anchors;
};

const FaceDetector::ModelConfig FaceDetector::FrontModel =
{
  name : "models/face_detection_front.tflite",
  options :
  {
    num_layers : 4,
    input_size_height : 128,
    input_size_width : 128,
    anchor_offset_x : 0.5f,
    anchor_offset_y : 0.5f,
    strides : {8, 16, 16, 16},
    interpolated_scale_aspect_ratio: 1.0f
  }
};

const FaceDetector::ModelConfig FaceDetector::BackModel =
{
  name : "models/face_detection_back.tflite",
  options :
  {
      num_layers: 4,
      input_size_height: 256,
      input_size_width: 256,
      anchor_offset_x: 0.5f,
      anchor_offset_y: 0.5f,
      strides: {16, 32, 32, 32},
      interpolated_scale_aspect_ratio: 1.0f
  }
};

const FaceDetector::ModelConfig FaceDetector::ShortModel =
{
  name : "models/face_detection_short_range.tflite",
  options : 
  {
    num_layers: 4,
    input_size_height: 128,
    input_size_width: 128,
    anchor_offset_x: 0.5f,
    anchor_offset_y: 0.5f,
    strides: {8, 16, 16, 16},
    interpolated_scale_aspect_ratio: 1.0f
  }
};

const FaceDetector::ModelConfig FaceDetector::FullModel =
{
  name : "models/face_detection_full_range.tflite",
  options : 
  {
    num_layers: 1,
    input_size_height: 192,
    input_size_width: 192,
    anchor_offset_x: 0.5f,
    anchor_offset_y: 0.5f,
    strides: {4},
    interpolated_scale_aspect_ratio: 0.0f
  }
};

const FaceDetector::ModelConfig FaceDetector::FullSparseModel =
{
  name : "models/face_detection_full_range_sparse.tflite",
  options : 
  {
    num_layers: 1,
    input_size_height: 192,
    input_size_width: 192,
    anchor_offset_x: 0.5f,
    anchor_offset_y: 0.5f,
    strides: {4},
    interpolated_scale_aspect_ratio: 0.0f
  }
};