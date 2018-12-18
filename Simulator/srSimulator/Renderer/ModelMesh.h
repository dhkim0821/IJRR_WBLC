#ifndef ModelMesh_H
#define ModelMesh_H

#include <stdio.h>
#include <vector>

// I decided to use my GLTexture class b/c adding all of its functions
// Would have greatly bloated the model class's code
// Just replace this with your favorite texture class
#include "srg/srgL.h"

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <map>
#include "Renderer/Texture.h"

class ModelMesh  
{
    private:
        std::string file_path_;
        void set_float4(float f[4], float a, float b, float c, float d);
        void color4_to_float4(const aiColor4D *c, float f[4]);
        void Color4f(const aiColor4D *color);
        void apply_material(const struct aiMaterial *mtl);
        void recursive_render (const struct aiScene *sc, const struct aiNode* nd);
        int LoadGLTextures(const aiScene* scene);
        const struct aiScene* scene;
        bool loadPngImage(const char *name, int &outWidth, int &outHeight, bool &outHasAlpha, GLubyte **outData);

        std::map<std::string, GLuint*> textureIdMap;	// map image filenames to textureIds
        GLuint*		textureIds;							// pointer to texture Array


        GLuint scene_list;
   
    public:
        SE3 LocalFrame;
        // Transformation
        float _T[16];
        bool	visible;			// True: the model gets rendered
        Vec3 mesh_scale_;

    public:
        ModelMesh();		// Constructor
        virtual ~ModelMesh();		// Destructor

        void    Load(char *name);	// Loads a model
        void	Draw();				// Draws the model
};

#endif
