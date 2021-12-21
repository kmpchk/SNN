#ifndef SNN_DRAWER_H
#define SNN_DRAWER_H

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>

typedef struct _Pos
{
    std::string name;
    int posx;
    int posy;
} Pos;

class Drawer {
public:

    Drawer(float, float, float, float, int);

    float radius;
    float margin;
    float distx;
    float disty;
    int line_width;
    int char_size = 26;
    const int levels = 5;
    int size_x = 200;
    int size_y = 200;
    std::vector<Pos> nodes;
    std::vector<int> numbers;
    sf::Font font;

    sf::RenderTexture texture;
    void save(std::string name);
    void draw();
    void init_texture();
    void add_node(int level, std::string text);
    void connect_nodes(std::string name1, std::string name2);
};

#endif