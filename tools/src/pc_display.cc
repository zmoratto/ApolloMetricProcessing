#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <osgViewer/Viewer>
#include <vw/Core/ProgressCallback.h>
#include <vw/Math.h>
#include <iostream>
#include <fstream>
#include <osg/LOD>

using namespace vw;

void reorder_array( osg::Vec3Array* array ) {
  // Reorganizing array so partial loading looks good.
  osg::Vec3Array* vertices = new osg::Vec3Array;
  vertices->reserve( array->size() );

  // I will be using divisors 16, 8, 4, 2, 1 (5 levels)
  for ( int32 i = 4; i >= 0; --i ) {
    uint32 divisor = 1 << i;
    uint32 hidivisor = 1 << (i+1);
    for ( uint32 j = 0; j < array->size(); j+=divisor ) {
      if ( (i != 4) && ((j%hidivisor==0) || (j == 0)) )
        continue;
      vertices->push_back( (*array)[j] );
    }
  }
  array = vertices; // delete old and replace with mine
}

osg::Node* point_cloud_lod( osg::Vec3Array* array ) {
  reorder_array( array );

  osg::LOD* lod = new osg::LOD;
  lod->setCenterMode( osg::LOD::USE_BOUNDING_SPHERE_CENTER );
  lod->setRangeMode(  osg::LOD::PIXEL_SIZE_ON_SCREEN );
  for ( uint32 i = 0; i < 5; i++ ) {
    uint32 divisor = 1 << i;
    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexArray(array);
    geometry->addPrimitiveSet( new osg::DrawArrays(GL_POINTS,0,
                                                   array->size()/divisor));
    osg::Vec4Array* color = new osg::Vec4Array(1);
    (*color)[0].set(1.0f,1.0f,1.0f,0.5f);
    geometry->setColorArray( color );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geometry);
    lod->addChild( geode, i == 4 ? 4 : 1600/(divisor*2),
                   i == 0 ? 1e6 : 1600/divisor );
  }

  return lod;
}

class KeyboardEventHandler : public osgGA::GUIEventHandler {
  osg::Group* m_scene;
  uint32 m_num_iterations, m_iteration, m_num_points;
  std::string m_point_file;
  std::vector<std::streampos> m_load_points;
public:
  KeyboardEventHandler(osg::Group* scene, uint32 iterations,
                       uint32 points, std::string point_file,
                       std::vector<std::streampos> load_points ) :
    m_scene(scene), m_num_iterations(iterations), m_iteration(0), m_num_points(points), m_point_file(point_file), m_load_points(load_points)  {}

  virtual bool handle( const osgGA::GUIEventAdapter& ea,
                       osgGA::GUIActionAdapter& ) {
    if ( ea.getEventType() == osgGA::GUIEventAdapter::KEYUP ) {
      switch ( ea.getKey() ) {
      case 'b':
      case 'B':
        m_iteration++;
        m_iteration %= m_num_iterations;
        break;
      case 'z':
      case 'Z':
        if ( m_iteration == 0 ) {
          m_iteration = m_num_iterations-1;
        } else {
          m_iteration--;
        }
        break;
      case '0': m_iteration = 0; break;
      case '1': m_iteration = (m_num_iterations-1)/9; break;
      case '2': m_iteration = (m_num_iterations*2-2)/9; break;
      case '3': m_iteration = (m_num_iterations*3-3)/9; break;
      case '4': m_iteration = (m_num_iterations*4-4)/9; break;
      case '5': m_iteration = (m_num_iterations*5-5)/9; break;
      case '6': m_iteration = (m_num_iterations*6-6)/9; break;
      case '7': m_iteration = (m_num_iterations*7-7)/9; break;
      case '8': m_iteration = (m_num_iterations*8-8)/9; break;
      case '9': m_iteration = m_num_iterations-1; break;
      default:
        return false;
      }
      scene_update();
      return true;
    } else {
      return false;
    }
  }

  void update_lod_array( osg::Node* node, osg::Vec3Array* array ) {
    osg::LOD* lod = dynamic_cast<osg::LOD*>(node);
    for ( uint32 i = 0; i < lod->getNumChildren(); i++ ) {
      osg::Geode* geode = dynamic_cast<osg::Geode*>(lod->getChild(i));
      if ( geode->getNumDrawables() != 1 ) {
        std::cout << "Geode Read Error!\n";
        continue;
      }
      osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode->getDrawable(0));
      geometry->setVertexArray(array);
      geometry->dirtyDisplayList();
    }
  }

  void scene_update() {
    std::ifstream file( m_point_file.c_str(), std::ios::in );
    if ( m_iteration > 0 )
      file.seekg(m_load_points[m_iteration-1]);
    osg::Vec3Array* vertices = new osg::Vec3Array;
    uint32 packet_vertices = 1024*8, packet_count = 0;
    vertices->reserve( packet_vertices );

    uint32 node_count = 0;

    for ( uint32 i = 0; i < m_num_points; i++ ) {
      osg::Vec3 pos; uint32 buffer;
      file >> buffer >> pos[0] >> pos[1] >> pos[2];
      if ( buffer != i )
        std::cout << "Read Error!";
      vertices->push_back(pos);

      packet_count++;
      if ( packet_count >= packet_vertices ) {
        // Update LOD
        update_lod_array( m_scene->getChild(node_count), vertices );

        // Reset for next loading
        vertices = new osg::Vec3Array;
        vertices->reserve( packet_vertices );
        packet_count = 0;
        node_count++;
      }
    }
    // Update LOD
    update_lod_array( m_scene->getChild(node_count), vertices );
    file.close();
  }
};

int main( int argc, char* argv[] ) {
  if ( argc == 1 ) {
    std::cerr << "Missing input file.\n";
    return 1;
  }

  std::string pntfile( argv[1] );
  std::vector<std::streampos> load_points;
  uint32 num_lines = 0, num_points = 0, num_timeiter = 0;

  osg::Group* scene = new osg::Group;
  { // Load file
    std::cout << "Indexing Point file ... \n";
    std::ifstream file( pntfile.c_str(), std::ios::in );
    char c;
    while (!file.eof()) {
      c=file.get();
      if (c == '\n'){
        std::streampos line_location = file.tellg();
        uint32 buffer;
        num_lines++;
        file >> buffer;
        if ( buffer == 0 )
          load_points.push_back(line_location);
        if ( buffer > num_points )
          num_points = buffer;
      }
    }
    num_points++;
    num_timeiter = num_lines/num_points;

    std::cout << "Number of points found: " << num_points << "\n";
    std::cout << "Load points......\n";
    for ( uint32 i = 0; i < load_points.size(); i++ )
      std::cout << "\t" << load_points[i] << "\n";

    file.clear();
    file.seekg(0);

    uint32 packet_vertices = 1024*8, packet_count = 0;;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->reserve( packet_vertices );

    // Load only the first iteration
    TerminalProgressCallback tpc("","Loading:");
    double tpc_inc = 1.0/double(num_points);
    for ( uint32 i = 0; i < num_points; ++i ) {
      tpc.report_incremental_progress(tpc_inc);
      osg::Vec3 pos;
      uint32 buffer;
      file >> buffer >> pos[0] >> pos[1] >> pos[2];
      if ( buffer != i )
        std::cout << "Read Error!";
      vertices->push_back(pos);

      packet_count++;
      if ( packet_count >= packet_vertices ) {
        scene->addChild( point_cloud_lod( vertices ) );
        vertices = new osg::Vec3Array;
        vertices->reserve( packet_vertices );
        packet_count = 0;
      }
    }
    tpc.report_finished();

    scene->addChild( point_cloud_lod( vertices ) );
    file.close();
  }

  { // Making the points large
    osg::StateSet* stateSet = new osg::StateSet();
    osg::Point* point = new osg::Point();
    point->setSize( 2.0f );
    stateSet->setAttribute( point );
    stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    stateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
    scene->setStateSet( stateSet );
  }

  { // Set the back drop
    osg::ClearNode* backdrop = new osg::ClearNode;
    backdrop->setClearColor( osg::Vec4f(0.0f,0.0f,0.0f,1.0f) );
    scene->addChild(backdrop);
  }

  osgViewer::Viewer viewer;
  viewer.addEventHandler( new KeyboardEventHandler(scene, num_timeiter, num_points,
                                                   pntfile, load_points) );
  viewer.setSceneData( scene );

  { // Setting up windoww
    osg::ref_ptr<osg::GraphicsContext::Traits> traits =
      new osg::GraphicsContext::Traits;
    traits->x = 100;
    traits->y = 100;
    traits->width = 1280;
    traits->height = 1024;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    traits->windowName = "Point Cloud Viewer";
    traits->vsync = true;
    traits->useMultiThreadedOpenGLEngine = false;
    traits->supportsResize = true;
    traits->useCursor = true;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext(gc.get());
    camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
    GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer(buffer);
    camera->setReadBuffer(buffer);

    viewer.addSlave(camera.get());
  }

  return viewer.run();
}
