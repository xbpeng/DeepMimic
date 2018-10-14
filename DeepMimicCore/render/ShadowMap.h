#pragma once

#include "TextureDesc.h"

class cShadowMap : public cTextureDesc
{
public:
	cShadowMap(void);
	~cShadowMap(void);

	bool Init(unsigned int WindowWidth, unsigned int WindowHeight);
	void Reshape(int w, int h);

private:
};

