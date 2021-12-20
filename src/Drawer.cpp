#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <string>
#include "Drawer.h"
#include <vector>
#include <iostream>
#include "spdlog/spdlog.h"

Drawer::Drawer()
{
    this->texture;
    texture.create(size_x, size_y);
    if (!font.loadFromFile("../../../res/calibri.ttf")){
        std::cout << "font not found";
    }
}

void Drawer::add_node(int level, int number, std::string text)
{
    sf::CircleShape shape(radius);
    int posx = margin + (dist + radius * 2) * (number - 1);
    int posy = margin + (dist + radius * 2) * level;

    shape.setFillColor(sf::Color::White);
    shape.setPosition(posx, size_x - posy);
    texture.draw(shape);

    sf::Text label;
    label.setFont(font);
    label.setString(text);
    label.setColor(sf::Color::Black);
    label.setCharacterSize(32);
    label.setScale(1.f, -1.f);
    label.setPosition(posx, size_x - posy + radius + margin * 2);
    texture.draw(label);

    Pos pos;
    pos.posx = posx;
    pos.posy = size_x - posy;
    pos.name = text;
    nodes.emplace_back(pos);
}

void Drawer::connect_nodes(std::string name1, std::string name2)
{
    Pos posa;
    Pos posb;
    for (int i=0; i < nodes.size(); i++) {
        if (nodes[i].name == name1){
            posa = nodes[i];
        }
        else if (nodes[i].name == name2)
        {
            posb = nodes[i];
        }
        //spdlog::debug("[draw x = {0}, y = {1}, name = {2}", nodes[i].posx, nodes[i].posy, nodes[i].name);
    }
    
    //spdlog::debug("[a x = {0}, y = {1}, name = {2}", posa.posx, posa.posy, posa.name);
    //spdlog::debug("[a x = {0}, y = {1}, name = {2}", posb.posx, posb.posy, posb.name);
    
    sf::VertexArray lines(sf::Lines, 2);

    lines[0].position = sf::Vector2f(posa.posx + radius, posa.posy + radius);
    lines[1].position = sf::Vector2f(posb.posx + radius, posb.posy + radius);
    lines[0].color = sf::Color::Red;
    lines[1].color = sf::Color::Blue;
    texture.draw(lines);
}

void Drawer::save(std::string name)
{
    texture.getTexture().copyToImage().saveToFile(name);
}