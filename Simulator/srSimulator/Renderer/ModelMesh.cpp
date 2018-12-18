#include <iostream>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "LieGroup/LieGroup.h"
#include "Renderer/ModelMesh.h"
#include <srConfiguration.h>
#include <libpng/png.h>
#include <sstream>
// This is used to generate a warning from the compiler
#define _QUOTE(x) # x
#define QUOTE(x) _QUOTE(x)
#define __FILE__LINE__ __FILE__ "(" QUOTE(__LINE__) ") : "
#define warn( x )  message( __FILE__LINE__ #x "\n" ) 

#define SAFE_DELETE(p) { if(p) { delete (p); (p)=NULL; } }
#define SAFE_DELETE_ARRAY(p) { if(p) { delete[] (p); (p)=NULL; } }

using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ModelMesh::ModelMesh()
{
    // Initialization
    scene_list = 0;

    // The model is visible by default
    visible = true;
}

ModelMesh::~ModelMesh() {
    //aiReleaseImport(scene);
    textureIdMap.clear(); //no need to delete pointers in it manually here. (Pointers point to textureIds deleted in next step)
}


void ModelMesh::Load(char *name)
{
    scene = aiImportFile(name, aiProcessPreset_TargetRealtime_MaxQuality);
    if(scene){
    }else{ printf("error: %s \n", name); }

    std::string full_path(name);
    std::size_t found = full_path.find_last_of("/");
    file_path_ = full_path.substr(0, found) + "/";

    LoadGLTextures(scene);
    glEnable(GL_TEXTURE_2D);
    //glShadeModel(GL_FLAT);
    glShadeModel(GL_SMOOTH);
    
    Draw();
    
    aiReleaseImport(scene);
}

void ModelMesh::apply_material(const struct aiMaterial *mtl)
{
    float c[4];

    GLenum fill_mode;
    int ret1, ret2;
    //struct aiColor4D diffuse;
    //struct aiColor4D specular;
    //struct aiColor4D ambient;
    //struct aiColor4D emission;

    aiColor4D diffuse;
    aiColor4D specular;
    aiColor4D ambient;
    aiColor4D emission;
    ai_real shininess, strength;
    int two_sided;
    int wireframe;
    unsigned int max;

    int texIndex = 0;
    aiString texPath;	//contains filename of texture
         if(AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath))
        {
            unsigned int texId = *textureIdMap[texPath.data];
            glBindTexture(GL_TEXTURE_2D, texId);
        }
   //if(scene->HasTextures()){
        //if(AI_SUCCESS == mtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath))
        //{
            //unsigned int texId = *textureIdMap[texPath.data];
            //glBindTexture(GL_TEXTURE_2D, texId);
        //}
    //}

    set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
        color4_to_float4(&diffuse, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

    set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
        color4_to_float4(&specular, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

    set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
        color4_to_float4(&ambient, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

    set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
        color4_to_float4(&emission, c);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

    max = 1;
    ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
    if(ret1 == AI_SUCCESS) {
        max = 1;
        ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
        if(ret2 == AI_SUCCESS)
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
        else
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }
    else {
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
        set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
    }

    max = 1;
    if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
        fill_mode = wireframe ? GL_LINE : GL_FILL;
    else
        fill_mode = GL_FILL;
    glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

    max = 1;
    if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
        glDisable(GL_CULL_FACE);
    else
        glEnable(GL_CULL_FACE);
}


/* ---------------------------------------------------------------------------- */
void ModelMesh::recursive_render (const struct aiScene *sc, const struct aiNode* nd)
{
    unsigned int i;
    unsigned int n = 0, t;
    aiMatrix4x4 m = nd->mTransformation;

    //printf("node coordinate\n");
    //std::cout<<m[0][0]<<", "<< m[0][1]<<", "<< m[0][2]<<", "<< m[0][3] <<std::endl;
    //std::cout<<m[1][0]<<", "<< m[1][1]<<", "<< m[1][2]<<", "<< m[1][3] <<std::endl;
    //std::cout<<m[2][0]<<", "<< m[2][1]<<", "<< m[2][2]<<", "<< m[2][3] <<std::endl;
    //std::cout<<m[3][0]<<", "<< m[3][1]<<", "<< m[3][2]<<", "<< m[3][3] <<std::endl;
    //printf("\n");
        //aiMatrix4x4 local_frame;
    //double Array[16];
    //(LocalFrame).ToArray(Array);
    //for(int i(0); i<4; ++i){
        //for(int j(0); j<4; ++j){
            //local_frame[j][i] = Array[j + 4*i];
        //}
    //}
    //printf("local coordinate\n");
    //std::cout<<local_frame[0][0]<<", "<< local_frame[0][1]<<", "<< local_frame[0][2]<<", "<< local_frame[0][3] <<std::endl;
    //std::cout<<local_frame[1][0]<<", "<< local_frame[1][1]<<", "<< local_frame[1][2]<<", "<< local_frame[1][3] <<std::endl;
    //std::cout<<local_frame[2][0]<<", "<< local_frame[2][1]<<", "<< local_frame[2][2]<<", "<< local_frame[2][3] <<std::endl;
    //std::cout<<local_frame[3][0]<<", "<< local_frame[3][1]<<", "<< local_frame[3][2]<<", "<< local_frame[3][3] <<std::endl;
    //printf("\n");
 
    //m = m* local_frame;
    //m = local_frame*m;
    /* update transform */
    aiTransposeMatrix4(&m);
    glPushMatrix();
    glPushMatrix();
    glMultMatrixf((float*)&m);
    /* draw all meshes assigned to this node */
    for (; n < nd->mNumMeshes; ++n) {
        const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];

        apply_material(sc->mMaterials[mesh->mMaterialIndex]);

        if(mesh->mNormals == NULL) {
            glDisable(GL_LIGHTING);
        } else {
            glEnable(GL_LIGHTING);
        }

        for (t = 0; t < mesh->mNumFaces; ++t) {
            const struct aiFace* face = &mesh->mFaces[t];
            GLenum face_mode;

            switch(face->mNumIndices) {
                case 1: face_mode = GL_POINTS; break;
                case 2: face_mode = GL_LINES; break;
                case 3: face_mode = GL_TRIANGLES; break;
                default: face_mode = GL_POLYGON; break;
            }

            glBegin(face_mode);
            for(i = 0; i < face->mNumIndices; i++)		// go through all vertices in face
            {
                int vertexIndex = face->mIndices[i];	// get group index for current index
                if(mesh->HasTextureCoords(0))		//HasTextureCoords(texture_coordinates_set)
                {
                    //mTextureCoords[channel][vertex]
                    //glTexCoord2f(mesh->mTextureCoords[0][vertexIndex].x, 
                    //1 - mesh->mTextureCoords[0][vertexIndex].y); 
                    glTexCoord2f(mesh->mTextureCoords[0][vertexIndex].x, 
                            mesh->mTextureCoords[0][vertexIndex].y); 
                }

                if(mesh->mColors[0] != NULL)  Color4f(&mesh->mColors[0][vertexIndex]);
                if(mesh->mNormals != NULL) {
                    glNormal3fv(&mesh->mNormals[vertexIndex].x);
                    //printf("normal exist\n");
                }
                glVertex3fv(&mesh->mVertices[vertexIndex].x);
                
            }

            //for(i = 0; i < face->mNumIndices; i++) {
            //int index = face->mIndices[i];
            //if(mesh->mColors[0] != NULL)
            //glColor4fv((GLfloat*)&mesh->mColors[0][index]);
            //if(mesh->mNormals != NULL)
            //glNormal3fv(&mesh->mNormals[index].x);
            //glVertex3fv(&mesh->mVertices[index].x);
            //}

            glEnd();
        }

    }

    /* draw all children */
    for (n = 0; n < nd->mNumChildren; ++n) {
        recursive_render(sc, nd->mChildren[n]);
    }
    //glPopMatrix();
}

void ModelMesh::Color4f(const aiColor4D *color)
{
    glColor4f(color->r, color->g, color->b, color->a);
}


void ModelMesh::Draw()
{
    if (visible)
    {
        glPushMatrix();
        glMultMatrixf(_T);
        //glMatrixMode(GL_MODELVIEW);
        //glLoadIdentity();

        //std::cout<<mesh_scale_<<std::endl;
        glScalef(mesh_scale_[0], mesh_scale_[1], mesh_scale_[2]);
        //glScalef(0.001f, 0.001f, 0.001f);
        glEnable(GL_NORMALIZE);
        if(scene_list == 0) {
            scene_list = glGenLists(1);
            glNewList(scene_list, GL_COMPILE);
            recursive_render(scene, scene->mRootNode);
            //glEndList();
        }
        //glCallList(scene_list);
    //glutSwapBuffers();
        glPopMatrix();
    }
}



int ModelMesh::LoadGLTextures(const aiScene* scene)
{
    /* getTexture Filenames and Numb of Textures */
    for (unsigned int m=0; m<scene->mNumMaterials; m++)
    {
        int texIndex = 0;
        aiReturn texFound = AI_SUCCESS;

        aiString path;	// filename

        while (texFound == AI_SUCCESS)
        {
            texFound = scene->mMaterials[m]->
                GetTexture(aiTextureType_DIFFUSE, texIndex, &path);
            textureIdMap[path.data] = NULL; //fill map with textures, pointers still NULL yet
            texIndex++;
        }
    }
    int numTextures = textureIdMap.size();

    /* create and fill array with GL texture ids */
    textureIds = new GLuint[numTextures];
    glGenTextures(numTextures, textureIds); /* Texture name generation */

    /* get iterator */
    std::map<std::string, GLuint*>::iterator itr = textureIdMap.begin();

    //std::string basepath = getBasePath(modelpath);
    for (int i=0; i<numTextures; i++)
    {
        //save IL image ID
        std::string filename = (*itr).first;  // get filename
        (*itr).second =  &textureIds[i];	  // save texture id for filename in map
        itr++;								  // next texture


        //std::cout<<filename<<std::endl;
        std::size_t found = filename.find_last_of(".");
        std::string tmp =  filename.substr(0, found);
        // change all image reference to png
        filename = tmp + ".png";
        std::string fileloc = file_path_ + filename;	/* Loading of image */

        int width, height;
        GLubyte *textureImage;
        bool hasAlpha;
        const char* name = fileloc.c_str();
        bool success = loadPngImage(name, width, height, hasAlpha, &textureImage);

        if (success) /* If no error occurred: */
        {
            // Convert every colour component into unsigned byte.If your image contains
            // alpha channel you can replace IL_RGB with IL_RGBA
            //success = ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);
            //if (!success)
            //{
            //abortGLInit("Couldn't convert image");
            //return -1;
            //}

            // Binding of texture name
            glBindTexture(GL_TEXTURE_2D, textureIds[i]);
            // redefine standard texture values
            // We will use linear interpolation for magnification filter
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
            // We will use linear interpolation for minifying filter
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
            // Texture specification
            //glTexImage2D(GL_TEXTURE_2D, 0, 
            //ilGetInteger(IL_IMAGE_BPP), 
            //ilGetInteger(IL_IMAGE_WIDTH),
            //ilGetInteger(IL_IMAGE_HEIGHT), 0, 
            //ilGetInteger(IL_IMAGE_FORMAT), GL_UNSIGNED_BYTE,
            //ilGetData());

            glTexImage2D(GL_TEXTURE_2D, 0, 
                    3, 
                    width, height, 0, 
                    GL_RGBA, GL_UNSIGNED_BYTE,
                    textureImage);
            // we also want to be able to deal with odd texture dimensions
            glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
            glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
            glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0 );
            glPixelStorei( GL_UNPACK_SKIP_ROWS, 0 );
        }
        else
        {
            if(filename != ".png"){
                printf("error during texture reading: %s\n", filename.c_str());
            }
            /* Error occurred */
            //MessageBox(NULL, ("Couldn't load Image: " + fileloc).c_str() , "ERROR", MB_OK | MB_ICONEXCLAMATION);
        }
    }
    // Because we have already copied image data into texture data  we can release memory used by image.
    //ilDeleteImages(numTextures, imageIds);

    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // Cleanup
    //delete [] imageIds;
    //imageIds = NULL;

    return TRUE;
}

bool ModelMesh::loadPngImage(const char *name, int &outWidth, int &outHeight, bool &outHasAlpha, GLubyte **outData) {
    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;
    int color_type, interlace_type;
    FILE *fp;

    if ((fp = fopen(name, "rb")) == NULL)
        return false;

    /* Create and initialize the png_struct
     * with the desired error handler
     * functions.  If you want to use the
     * default stderr and longjump method,
     * you can supply NULL for the last
     * three parameters.  We also supply the
     * the compiler header file version, so
     * that we know if the application
     * was compiled with a compatible version
     * of the library.  REQUIRED
     */
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING,
            NULL, NULL, NULL);

    if (png_ptr == NULL) {
        fclose(fp);
        return false;
    }

    /* Allocate/initialize the memory
     * for image information.  REQUIRED. */
    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        fclose(fp);
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return false;
    }

    /* Set error handling if you are
     * using the setjmp/longjmp method
     * (this is the normal method of
     * doing things with libpng).
     * REQUIRED unless you  set up
     * your own error handlers in
     * the png_create_read_struct()
     * earlier.
     */
    if (setjmp(png_jmpbuf(png_ptr))) {
        /* Free all of the memory associated
         * with the png_ptr and info_ptr */
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        /* If we get here, we had a
         * problem reading the file */
        return false;
    }

    /* Set up the output control if
     * you are using standard C streams */
    png_init_io(png_ptr, fp);

    /* If we have already
     * read some of the signature */
    png_set_sig_bytes(png_ptr, sig_read);

    /*
     * If you have enough memory to read
     * in the entire image at once, and
     * you need to specify only
     * transforms that can be controlled
     * with one of the PNG_TRANSFORM_*
     * bits (this presently excludes
     * dithering, filling, setting
     * background, and doing gamma
     * adjustment), then you can read the
     * entire image (including pixels)
     * into the info structure with this
     * call
     *
     * PNG_TRANSFORM_STRIP_16 |
     * PNG_TRANSFORM_PACKING  forces 8 bit
     * PNG_TRANSFORM_EXPAND forces to
     *  expand a palette into RGB
     */
    png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_STRIP_16 | PNG_TRANSFORM_PACKING | PNG_TRANSFORM_EXPAND, NULL);

    png_uint_32 width, height;
    int bit_depth;
    png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type,
            &interlace_type, NULL, NULL);
    outWidth = width;
    outHeight = height;

    unsigned int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
    *outData = (unsigned char*) malloc(row_bytes * outHeight);

    png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);

    for (int i = 0; i < outHeight; i++) {
        // note that png is ordered top to
        // bottom, but OpenGL expect it bottom to top
        // so the order or swapped
        memcpy(*outData+(row_bytes * (outHeight-1-i)), row_pointers[i], row_bytes);
    }

    /* Clean up after the read,
     * and free any memory allocated */
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

    /* Close the file */
    fclose(fp);

    /* That's it */
    return true;
}


/* ---------------------------------------------------------------------------- */
void ModelMesh::color4_to_float4(const aiColor4D *c, float f[4])
{
    f[0] = c->r;
    f[1] = c->g;
    f[2] = c->b;
    f[3] = c->a;
}

/* ---------------------------------------------------------------------------- */
void ModelMesh::set_float4(float f[4], float a, float b, float c, float d)
{
    f[0] = a;
    f[1] = b;
    f[2] = c;
    f[3] = d;
}


