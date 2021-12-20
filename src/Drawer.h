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

    Drawer();

    const float radius = 140.f;
    const float margin = 10.f;
    const float dist = 100.f;
    int size_x = 2000;
    int size_y = 2000;
    std::vector<Pos> nodes;
    sf::Font font;

    sf::RenderTexture texture;
    void save(std::string name);
    void add_node(int level, int number, std::string text);
    void connect_nodes(std::string name1, std::string name2);
};

#endif