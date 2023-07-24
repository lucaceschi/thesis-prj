#include <Eigen/Dense>
#include "framework/app.hpp"
#include "framework/debug.hpp"
#include <imgui.h>
#include <wrap/gui/trackball.h>

#include <stdio.h>


class MainApp : public frmwrk::App
{
public:
	MainApp()
        : App("MainApp", Eigen::Vector2i{500, 500}, true)
    {}

private:
    Eigen::Vector3f bgColor_ = {0.18f, 0.18f, 0.18f};
    bool picking_ = false;

    GLdouble pointsPos_[5][3] = {
        { 0.0,  0.0,  0.0},
        { 0.5,  0.0,  0.0},
        {-0.5,  0.0,  0.0},
        { 0.0,  0.5,  0.0},
        { 0.0, -0.5,  0.0}
    };

	virtual bool initApp()
    {
        ImGui::StyleColorsLight();
        ImGuiIO* io = &ImGui::GetIO();
        io->Fonts->AddFontFromFileTTF("../external/imgui/misc/fonts/Karla-Regular.ttf", 15);
        return true;
    }

    void handleInput()
    {
        if(!ImGui::GetIO().WantCaptureMouse)
        {
            picking_ = false;
            if(input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
                picking_ = true;
        }
    }

    void draw(bool selectMode = false)
    {
        glPointSize(20.0f);
        for(int i = 0; i < 5; i++)
        {
            if(selectMode)
                glLoadName(i+1);
            
            glColor3f(i/5.0,((i+2)%5)/5.0,((i+3)%5)/5.0);
            glBegin(GL_POINTS);
            glVertex3d(pointsPos_[i][0], pointsPos_[i][1], pointsPos_[i][2]);
            glEnd();
        }
    }

    void list_hits(GLint hits, GLuint *names)
    {
        int i;
    
        frmwrk::Debug::logSuccess("%d hits:", hits);
    
        for (i = 0; i < hits; i++)
            frmwrk::Debug::log(	"Number: %d\n"
                        "Min Z: %d\n"
                        "Max Z: %d\n"
                        "Name on stack: %d\n",
            (GLubyte)names[i * 4],
            (GLubyte)names[i * 4 + 1],
            (GLubyte)names[i * 4 + 2],
            (GLubyte)names[i * 4 + 3]
                    );
    }

    void render()
    {        
        Eigen::Vector2i fbSize = getFramebufferSize();
        glViewport(0, 0, fbSize(0), fbSize(1));

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(-1, 1, -1, 1);

        if(picking_)
        {
            GLint viewport[4] = {0, 0, fbSize(0), fbSize(1)};
            GLuint hitRecords[64];
            Eigen::Vector2d curPos = input_.getCursorPos();
            Eigen::Vector2i winSize = getWindowSize();

            glSelectBuffer(64, hitRecords);
            glRenderMode(GL_SELECT);
            glInitNames();
            glPushName(~0U);

            glPushMatrix();
            gluPickMatrix(curPos(0), winSize(1)-curPos(1), 20, 20, viewport);

            draw(true);

            glFlush();
            GLint nHits = glRenderMode(GL_RENDER);
            glPopMatrix();

            processHits(nHits, hitRecords);
        }
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        draw();

        ImGui::Begin("My First Tool");
        ImGui::ColorEdit3("color", bgColor_.data());
        ImGui::End();
        glClearColor(bgColor_[0], bgColor_[1], bgColor_[2], 1.0f);
    }


    void processHits (GLint hits, GLuint buffer[])
    {
        unsigned int i, j;
        GLuint names, *ptr;

        printf ("hits = %d\n", hits);
        ptr = (GLuint *) buffer;
        for (i = 0; i < hits; i++) { /*  for each hit  */
            names = *ptr;
            printf (" number of names for hit = %d\n", names); ptr++;
            printf("  z1 is %g;", (float) *ptr/0x7fffffff); ptr++;
            printf(" z2 is %g\n", (float) *ptr/0x7fffffff); ptr++;
            printf ("   the name is ");
            for (j = 0; j < names; j++) {     /*  for each name */
                printf ("%d ", *ptr); ptr++;
            }
            printf ("\n");
        }
    }

	virtual bool mainLoop(double delta)
    {
        handleInput();
        render();

        return true;
    }
};


int main(int argc, char const *argv[])
{
	MainApp app = MainApp();
	app.run();

    fflush(stdin);

    return 0;
}
