"use strict";
var MyHW2 = function() {
    // MAIN DEFINITIONS ----------------------------------------------------------------------------------------------------------------------------------------------------------------
    var canvas;
    var gl;
    var program;

    // MATRICES
    var modelViewMatrix, projectionMatrix, nMatrix, instanceMatrix;
    var modelViewMatrixLoc;

    // COORDINATES ARRAYS
    var positionsArray = [];
    var texCoordsArray = [];
    var normalsArray = [];
    var tangArray = [];
    
    // BUFFERS
    var vBuffer, tBuffer, nBuffer, tanBuffer;

    // VERTICES TO BUILD CUBES
    var vertices = [
        vec4( -0.5, -0.5,  0.5, 1.0 ),
        vec4( -0.5,  0.5,  0.5, 1.0 ),
        vec4( 0.5,  0.5,  0.5, 1.0 ),
        vec4( 0.5, -0.5,  0.5, 1.0 ),
        vec4( -0.5, -0.5, -0.5, 1.0 ),
        vec4( -0.5,  0.5, -0.5, 1.0 ),
        vec4( 0.5,  0.5, -0.5, 1.0 ),
        vec4( 0.5, -0.5, -0.5, 1.0 )
    ];

    // CUBE CREATING FUNCTIONS 
    function quad(a, b, c, d) {
        var t1 = subtract(vertices[b], vertices[a]);
        var t2 = subtract(vertices[c], vertices[b]);
        var normal = cross(t1, t2);
        normal = vec3(normal);

        positionsArray.push(vertices[a]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[0]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[b]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[1]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[c]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[2]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));

        positionsArray.push(vertices[d]);
        normalsArray.push(normal);
        texCoordsArray.push(texCoord[3]);
        tangArray.push(vec3(t1[0], t1[1], t1[2]));
    }

    function cube(){
        quad( 1, 0, 3, 2 );
        quad( 2, 3, 7, 6 );
        quad( 3, 0, 4, 7 );
        quad( 6, 5, 1, 2 );
        quad( 4, 5, 6, 7 );
        quad( 5, 4, 0, 1 );
    }

    // CAMERA
    var at = vec3(0.0, 0.0, 0.0);
    var up = vec3(0.0, 1.0, 0.0);
    var radius = 12;
    var thetaCam = 15*Math.PI/180.0;
    var phi = 15*Math.PI/180.0;

    var left = -26;
    var right = 26;
    var bottom = -13;
    var top = 13;
    var near = -40.0;
    var far = 40.0;

    // LIGHT
    var lightPosition = vec4(4, 15, 0, 1.0);
    var lightAmbient = vec4(0.2, 0.2, 0.2, 1.0);
    var lightDiffuse = vec4(1, 1, 1, 1.0);
    var lightSpecular = vec4(1, 1, 1, 1.0);

    var materialAmbient = vec4(1, 1, 1, 1.0);
    var materialDiffuse = vec4(1, 1, 1, 1.0);
    var materialSpecular = vec4(0.7, 0.7, 0.7, 1.0);
    var materialShininess = 5.0;

    var ambientProduct = mult(lightAmbient, materialAmbient);
    var diffuseProduct = mult(lightDiffuse, materialDiffuse);
    var specularProduct = mult(lightSpecular, materialSpecular);

    // TEXTURES -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Flag used to determine if the texture images are correctly loaded inside the browser
    var okay = true;

    var texCoord = [
        vec2(0, 0),
        vec2(0, 1),
        vec2(1, 1),
        vec2(1, 0)
    ];
    
    var grassTexture;
    var grass_flag = true;
    var woolTexture;
    var wool_flag = true;
    var fenceTexture;
    var fence_flag = true;
    var bumpTexture;
    var bump_flag = true;
    var eye_flag = true;

    // BUMP TEXTURE CONSTRUCTION
    var texSize = 40;
    // Bump Data
    var data = new Array()
    for (var i = 0; i<= texSize; i++)  data[i] = new Array();
    for (var i = 0; i<= texSize; i++) for (var j=0; j<=texSize; j++)
        data[i][j] = Math.random()*0.4;
    // Bump Map Normals
    var normalst = new Array()
    for (var i=0; i<texSize; i++)  normalst[i] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++)
        normalst[i][j] = new Array();
    for (var i=0; i<texSize; i++) for ( var j = 0; j < texSize; j++) {
        normalst[i][j][0] = data[i][j]-data[i+1][j];
        normalst[i][j][1] = data[i][j]-data[i][j+1];
        normalst[i][j][2] = 1;
    }
    // Scale to Texture Coordinates
    for (var i=0; i<texSize; i++) for (var j=0; j<texSize; j++) {
        var d = 0;
        for(k=0;k<3;k++) d+=normalst[i][j][k]*normalst[i][j][k];
        d = Math.sqrt(d);
        for(k=0;k<3;k++) normalst[i][j][k]= 0.5*normalst[i][j][k]/d + 0.5;
    }
    // Normal Texture Array
    var normals = new Uint8Array(3*texSize*texSize);
    for ( var i = 0; i < texSize; i++ )
        for ( var j = 0; j < texSize; j++ )
            for(var k =0; k<3; k++)
                normals[3*texSize*i+3*j+k] = 255*normalst[i][j][k];

    // Configure the textures, in my case there are 4 textures: grass, wool, fence and bump.
    function configureTexture(image1, image2, image3, bump_img) {
        grassTexture = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, grassTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, image1);
        gl.generateMipmap(gl.TEXTURE_2D);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST_MIPMAP_LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.uniform1i(gl.getUniformLocation(program, "uGrassTextureMap"), 0);

        woolTexture = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, woolTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, image2);
        gl.generateMipmap(gl.TEXTURE_2D);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST_MIPMAP_LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.uniform1i(gl.getUniformLocation(program, "uWoolTextureMap"), 1);

        fenceTexture = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, fenceTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, image3);
        gl.generateMipmap(gl.TEXTURE_2D);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST_MIPMAP_LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.uniform1i(gl.getUniformLocation(program, "uFenceTextureMap"), 2);

        bumpTexture = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, bumpTexture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSize, texSize, 0, gl.RGB, gl.UNSIGNED_BYTE, bump_img);
        gl.generateMipmap(gl.TEXTURE_2D);
        gl.uniform1i(gl.getUniformLocation(program, "uBumpTextureMap"), 3);
    }

    //  HIERARCHICAL MODELS' NODES -------------------------------------------------------------------------------------------------------------------------------------------------------
    var numNodes = 19;
    
    // IDs IN THETA
    //--------BODY-HEAD-LEGS--------------------------TAIL--EYES--FENCE------------GRASS
    var theta = [0, 5, 180, 0, 180, 0, 180, 0, 180, 0, 315, 5, 5, 0, 0, 0, 90, 90, 0];
    var stack = [];
    var figure = [];

    // BODY: ID 0
    var bodyPos = [-15, 0, 0];
    var bodyId = 0;

    var bodyHeight = 3.0;
    var bodyWidth = 4.5;

    // HEAD: ID 1
    var headId  = 1;

    var headHeight = 1.8;
    var headWidth = 1.8;

    // LEGS: ID 2-9
    var leftFrontUpLegId = 2;
    var leftFrontLowLegId = 3;
    var rightFrontUpLegId = 4;
    var rightFrontLowLegId = 5;
    var leftRearUpLegId = 6;
    var leftRearLowLegId = 7;
    var rightRearUpLegId = 8;
    var rightRearLowLegId = 9;

    var upperLegWidth  = 1;
    var upperLegHeight = 1.1;
    var lowerLegWidth  = 0.8;
    var lowerLegHeight = 1.6;

    // TAIL: ID 10
    var tailId = 10;

    var tailHeight = 0.5;
    var tailWidth = 1.0;

    // EYES: ID 11-12
    var eyeId = 11;
    var eyeId2 = 12;

    var eyeWidth= 0.36;
    var eyeHeight= 0.36;

    // FENCE: ID 13-17
    var fencePos = [2, 0, 0]
    var fence1Id = 13;
    var fence2Id = 14;
    var fence3Id = 15;
    var fence4Id = 16;
    var fence5Id = 17;

    var fenceWidth = 0.7;
    var fenceHeight = 5.0;

    // GRASS FIELD: ID 18
    var grassId = 18;

    var grassWidth = 40;
    var grassHeight = 0.3;

    // NODES' FUNCTIONS ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    function scale4(a, b, c) {
        var result = mat4();
        result[0] = a;
        result[5] = b;
        result[10] = c;
        return result;
    }

    function createNode(transform, render, sibling, child){
        var node = {
            transform: transform,
            render: render,
            sibling: sibling,
            child: child,
        }
        return node;
    }

    function traverse(Id) {
        if(Id == null) return;
        stack.push(modelViewMatrix);
        modelViewMatrix = mult(modelViewMatrix, figure[Id].transform);
        figure[Id].render();
        if(figure[Id].child != null) traverse(figure[Id].child);
        modelViewMatrix = stack.pop();
        if(figure[Id].sibling != null) traverse(figure[Id].sibling);
    }

    // For each case: translate; rotate; specify render function, sibling and child
    // The cubes are then scaled in the render functions
    function initNodes(Id) {
        var m = mat4();
        var f = mat4();

        switch(Id) {
            case bodyId:
                m = translate(bodyPos[0], bodyPos[1], bodyPos[2]);
                m = mult(m, rotate(theta[bodyId], vec3(0, 1, 0)));
                figure[bodyId] = createNode( m, body, null, headId );
                break;

            case headId:
                m = translate(2.5, 2.2, 0.0);
                m = mult(m, rotate(theta[headId], vec3(0, 0, 1)));
                figure[headId] = createNode( m, head, leftFrontUpLegId, eyeId);
                break;

            case eyeId:
                m = translate(0.5, 1, 0.9);
                m = mult(m, rotate(theta[eyeId], vec3(0, 0, 1)));
                figure[eyeId] = createNode( m, eye, eyeId2, null );
                break; 

            case eyeId2:
                m = translate(0.5, 1, -0.9);
                m = mult(m, rotate(theta[eyeId2], vec3(0, 0, 1)));
                figure[eyeId2] = createNode( m, eye, null, null );
                break;

            case leftFrontUpLegId:
                m = translate(1.7, 0, -0.9);
                m = mult(m, rotate(theta[leftFrontUpLegId], vec3(0, 0, 1)));
                figure[leftFrontUpLegId] = createNode( m, upLeg, rightFrontUpLegId, leftFrontLowLegId);
                break;

            case rightFrontUpLegId:
                m = translate(1.7, 0, 0.9);
                m = mult(m, rotate(theta[rightFrontUpLegId], vec3(0, 0, 1)));
                figure[rightFrontUpLegId] = createNode( m, upLeg, leftRearUpLegId, rightFrontLowLegId);
                break;

            case leftRearUpLegId:
                m = translate(-1.7, 0, -0.9);
                m = mult(m , rotate(theta[leftRearUpLegId], vec3(0, 0, 1)));
                figure[leftRearUpLegId] = createNode( m, upLeg, rightRearUpLegId, leftRearLowLegId);
                break;

            case rightRearUpLegId:
                m = translate(-1.7, 0, 0.9);
                m = mult(m, rotate(theta[rightRearUpLegId], vec3(0, 0, 1)));
                figure[rightRearUpLegId] = createNode( m, upLeg, tailId, rightRearLowLegId);
                break;

            case leftFrontLowLegId:
                m = translate(0.0, upperLegHeight, 0.0);
                m = mult(m, rotate(theta[leftFrontLowLegId], vec3(0, 0, 1)));
                figure[leftFrontLowLegId] = createNode( m, lowLeg, null, null);
                break;

            case rightFrontLowLegId:
                m = translate(0.0, upperLegHeight, 0.0);
                m = mult(m, rotate(theta[rightFrontLowLegId], vec3(0, 0, 1)));
                figure[rightFrontLowLegId] = createNode( m, lowLeg, null, null);
                break;

            case leftRearLowLegId:
                m = translate(0.0, upperLegHeight, 0.0);
                m = mult(m, rotate(theta[leftRearLowLegId],vec3(0, 0, 1)));
                figure[leftRearLowLegId] = createNode( m, lowLeg, null, null);
                break;

            case rightRearLowLegId:
                m = translate(0.0, upperLegHeight, 0.0);
                m = mult(m, rotate(theta[rightRearLowLegId], vec3(0, 0, 1)));
                figure[rightRearLowLegId] = createNode( m, lowLeg, null, null);
                break;

            case tailId:
                m = translate(-2.2, 1.8, 0);
                m = mult(m, rotate(theta[tailId], vec3(0, 0, 1)));
                figure[tailId] = createNode( m, tail, null, null);
                break;

            case fence1Id:
                f = translate(fencePos[0], fencePos[1], fencePos[2]);
                f = mult(f, rotate(theta[fence1Id], vec3(1, 0, 0)));
                figure[fence1Id] = createNode( f, fence, fence2Id, null);
                break;

            case fence2Id:
                f = translate(2, 0, 3);
                f = mult(f, rotate(theta[fence2Id], vec3(1, 0, 0)));
                figure[fence2Id] = createNode( f, fence, fence3Id, null);
                break;

            case fence3Id:
                f = translate(2, 0, -3);
                f = mult(f, rotate(theta[fence3Id], vec3(1, 0, 0)));
                figure[fence3Id] = createNode( f, fence, fence4Id, null);
                break;

            case fence4Id:
                f = translate(2, 2, 2.5);
                f = mult(f, rotate(theta[fence4Id], vec3(1, 0, 0)));
                figure[fence4Id] = createNode( f, fenceHorizontal, fence5Id, null);
                break;
            
            case fence5Id:
                f = translate(2, 4, 2.5);
                f = mult(f, rotate(theta[fence5Id], vec3(1, 0, 0)));
                figure[fence5Id] = createNode( f, fenceHorizontal, null, null);
                break;     

            case grassId:
                f = translate(0, -2.7, 0);
                f = mult(f, rotate(theta[grassId], vec3(1, 0, 0)));
                figure[grassId] = createNode( f, grass, null, fence1Id);
                break;
        }
    }

    // RENDER FUNCTIONS ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // SHEEP
    function body() {
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = false;
        eye_flag = false;
        fence_flag = false;
        bump_flag = true;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5*bodyHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale( bodyWidth, bodyHeight, bodyHeight));
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    function head() {
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = true;
        eye_flag = false;
        fence_flag = false;
        bump_flag = false;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * headHeight, 0.0 ));
        instanceMatrix = mult(instanceMatrix, scale(headWidth, headHeight, headWidth) );
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    function eye(){
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = false;
        eye_flag = true;
        fence_flag = false;
        bump_flag = false;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate( 0.0, 0.5 * eyeHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale(eyeWidth, eyeHeight, eyeWidth) );
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    function lowLeg() {
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = true;
        eye_flag = false;
        fence_flag = false;
        bump_flag = false;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate( 0.0, 0.5 * lowerLegHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale(lowerLegWidth, lowerLegHeight, lowerLegWidth) );
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    function upLeg() {
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = false;
        eye_flag = false;
        fence_flag = false;
        bump_flag = true;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * upperLegHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale(upperLegWidth, upperLegHeight, upperLegWidth) );
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    function tail(){
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = false;
        eye_flag = false;
        fence_flag = false;
        bump_flag = true;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * tailHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale(tailWidth, tailHeight, tailWidth) )
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    // FENCE
    function fence(){
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = false;
        eye_flag = false;
        fence_flag = true;
        bump_flag = false;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * fenceHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale(fenceWidth, fenceHeight, fenceWidth))
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    function fenceHorizontal(){
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = false;
        wool_flag = false;
        eye_flag = false;
        fence_flag = true;
        bump_flag = false;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * fenceHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale(fenceWidth, fenceHeight+5, fenceWidth))
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }

    // GRASS
    function grass(){
        // Texture whose flag is true is applied to this specific part of model
        grass_flag = true;
        wool_flag = false;
        eye_flag = false;
        fence_flag = false;
        bump_flag = false;
        gl.uniform1f( gl.getUniformLocation(program, "uGrassFlag"), grass_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uWoolFlag"), wool_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uEyeFlag"), eye_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uFenceFlag"), fence_flag );
        gl.uniform1f( gl.getUniformLocation(program, "uBumpFlag"), bump_flag );

        instanceMatrix = mult(modelViewMatrix, translate(0.0, 0.5 * grassHeight, 0.0) );
        instanceMatrix = mult(instanceMatrix, scale(grassWidth, grassHeight, grassWidth))
        gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(instanceMatrix) );
        for(var i =0; i<6; i++) gl.drawArrays(gl.TRIANGLE_FAN, 4*i, 4);
    }



    // ANIMATION -----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // START ANIMATION
    var Animation;
    var step = true;
    var speed = 0.2;
    var jump_ended = false;
    
    function startAnimation(){
        // WALK: perpendicular -> step1 -> perp -> step2 -> perp -> step1 -> .......
        // Walk while approaching the fence OR take a few step after landing
        if (bodyPos[0] < -6 || (bodyPos[0] >= 9.4 && bodyPos[0] < 13)){
            // x increment
            bodyPos[0] += speed;
            // Acceleration: speed linearly increase
            speed += 0.05;
            initNodes(bodyId);
            // Legs go back to "perpendicular to ground" position after a step
            if(theta[leftRearUpLegId] != 180){
                theta[headId] = 12.5;  
                theta[leftRearUpLegId] = 180;    
                theta[rightRearUpLegId] = 180;
                theta[leftFrontUpLegId] = 180;
                theta[rightFrontUpLegId] = 180;
                theta[leftFrontLowLegId] = 0;
                theta[rightFrontLowLegId] = 0;

                initNodes(headId);
                initNodes(leftFrontUpLegId);
                initNodes(rightFrontUpLegId);
                initNodes(leftRearUpLegId);
                initNodes(rightRearUpLegId);
                initNodes(leftFrontLowLegId);
                initNodes(rightFrontLowLegId);
                return;
            }
            // Step type 1
            if (step){
                theta[headId]=20;
                theta[leftFrontUpLegId] += -25;
                theta[rightFrontUpLegId] += 25;
                theta[leftRearUpLegId] +=25;
                theta[rightRearUpLegId] +=-25;
                theta[leftFrontLowLegId] += 25;
                theta[rightFrontLowLegId] += 25;

                initNodes(headId);
                initNodes(leftFrontUpLegId);
                initNodes(rightFrontUpLegId);
                initNodes(leftRearUpLegId);
                initNodes(rightRearUpLegId);
                initNodes(leftFrontLowLegId);
                initNodes(rightFrontLowLegId);
                step = false;
            }
            // Step type 2
            else{
                theta[headId]=5;
                theta[leftFrontUpLegId] += 25;
                theta[rightFrontUpLegId] += -25;
                theta[leftRearUpLegId] +=-25;
                theta[rightRearUpLegId] +=25;
                theta[leftFrontLowLegId] += 25;
                theta[rightFrontLowLegId] += 25;
    
                initNodes(headId);
                initNodes(leftFrontUpLegId);
                initNodes(rightFrontUpLegId);
                initNodes(leftRearUpLegId);
                initNodes(rightRearUpLegId);
                initNodes(leftFrontLowLegId);
                initNodes(rightFrontLowLegId);
                step = true;
            }
        }

        // JUMP
        else{
            // If the jump ended AND the sheep is not walking anymore => animation has ended => the sheep stays still
            if(jump_ended){
                bodyPos[1] = 0;
                theta[leftRearUpLegId] = 180;    
                theta[rightRearUpLegId] = 180;
                theta[leftFrontUpLegId] = 180;
                theta[rightFrontUpLegId] = 180;
                theta[leftFrontLowLegId] = 0;
                theta[rightFrontLowLegId] = 0;

                initNodes(leftFrontUpLegId);
                initNodes(rightFrontUpLegId);
                initNodes(leftRearUpLegId);
                initNodes(rightRearUpLegId);
                initNodes(leftFrontLowLegId);
                initNodes(rightFrontLowLegId);

                return;
            }
            speed = 0.8
            // UP - TAKE OFF
            // x increment
            bodyPos[0] += speed;
            if(bodyPos[0] < fencePos[0]+0.5){
                // y increment
                bodyPos[1] += 0.5;
                theta[leftRearUpLegId] = 205;
                theta[rightRearUpLegId] = 205;
                theta[leftFrontUpLegId] = 155;
                theta[rightFrontUpLegId] = 155;
                theta[leftFrontLowLegId] = 90;
                theta[rightFrontLowLegId] = 90;

                initNodes(bodyId);
                initNodes(leftFrontUpLegId);
                initNodes(rightFrontUpLegId);
                initNodes(leftRearUpLegId);
                initNodes(rightRearUpLegId);
                initNodes(leftFrontLowLegId);
                initNodes(rightFrontLowLegId);
            }

            // DOWN - LANDING
            if(bodyPos[0]>fencePos[0]+0.5){
                // y decrement
                bodyPos[1] -= 0.5;
                theta[leftRearUpLegId] = 155;
                theta[rightRearUpLegId] = 155;
                theta[leftFrontUpLegId] = 155;
                theta[rightFrontUpLegId] = 155;
                theta[leftFrontLowLegId] = 0;
                theta[rightFrontLowLegId] = 0;

                initNodes(bodyId);
                initNodes(leftFrontUpLegId);
                initNodes(rightFrontUpLegId);
                initNodes(leftRearUpLegId);
                initNodes(rightRearUpLegId);
                initNodes(leftFrontLowLegId);
                initNodes(rightFrontLowLegId);
                // Jump ended when the landing phase takes the sheep very near to the ground
                if (bodyPos[1] < 0.6)                    
                    jump_ended = true;
            }
        }
    }

    // RESET ANIMATION
    function resetAnimation(){
        // Every useful variable is set to its original value
        theta = [0, 0, 180, 0, 180, 0, 180, 0, 180, 0, 5, -45, 5, 0, 0, 0, 90, 90, 0, 0, 5];
        bodyPos = [-15, 0, 0];
        step = true;
        speed = 0.2;
        jump_ended = false;

        theta[leftRearUpLegId] = 180;
        theta[rightRearUpLegId] = 180;
        theta[leftFrontUpLegId] = 180;
        theta[rightFrontUpLegId] = 180;
        theta[leftFrontLowLegId] = 0;
        theta[rightFrontLowLegId] = 0;
        theta[headId] = 5;

        initNodes(bodyId);
        initNodes(headId);
        initNodes(leftFrontUpLegId);
        initNodes(rightFrontUpLegId);
        initNodes(leftRearUpLegId);
        initNodes(rightRearUpLegId);
        initNodes(leftFrontLowLegId);
        initNodes(rightFrontLowLegId);

        // Animation is interrupted
        clearInterval(Animation);
    }

    // INIT ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    window.onload = function init(){

        canvas = document.getElementById( "gl-canvas" );

        gl = canvas.getContext('webgl2');
        if (!gl) { alert( "WebGL 2.0 isn't available" ); }

        gl.viewport( 0, 0, canvas.width, canvas.height );
        gl.clearColor( 0.5, 0.87, 1.0, 1.0 );

        //  Load shaders and initialize attribute buffers
        program = initShaders( gl, "vertex-shader", "fragment-shader");

        gl.useProgram( program);
        gl.enable(gl.DEPTH_TEST);

        // SLIDERS
        document.getElementById("depthSlider").onchange = function(event) {
            far = event.target.value/2;
            near = -event.target.value/2;
        };
        document.getElementById("heightSlider").onchange = function(event) {
            top = event.target.value/2;
            bottom = -event.target.value/2;
        };
        document.getElementById("widthSlider").onchange = function(event) {
            right = event.target.value/2;
            left = -event.target.value/2;
        };
        document.getElementById("radiusSlider").onchange = function(event) {radius = event.target.value;};
        document.getElementById("thetaSlider").onchange = function(event) {thetaCam = event.target.value* Math.PI/180.0;};
        document.getElementById("phiSlider").onchange = function(event) {phi = event.target.value* Math.PI/180.0;};

        cube();

        // POSITIONS BUFFER
        vBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, vBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(positionsArray), gl.STATIC_DRAW);
        var positionLoc = gl.getAttribLocation(program, "aPosition");
        gl.vertexAttribPointer(positionLoc, 4, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(positionLoc);

        // TEXTURES BUFFER
        tBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, tBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(texCoordsArray), gl.STATIC_DRAW);
        var texCoordLoc = gl.getAttribLocation(program, "aTexCoord");
        gl.vertexAttribPointer(texCoordLoc, 2, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(texCoordLoc);

        // NORMALS BUFFER
        nBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, nBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(normalsArray), gl.STATIC_DRAW);
        var normalLoc = gl.getAttribLocation(program, "aNormal");
        gl.vertexAttribPointer(normalLoc, 3, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(normalLoc);

        // TANGENTS BUFFER
        tanBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, tanBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, flatten(tangArray), gl.STATIC_DRAW);
        var tangentLoc = gl.getAttribLocation(program, "aTangent");
        gl.vertexAttribPointer(tangentLoc, 3, gl.FLOAT, false, 0, 0);
        gl.enableVertexAttribArray(tangentLoc);

        // ILLUMINATION
        gl.uniform4fv(gl.getUniformLocation(program, "uLightPosition"), lightPosition );
        gl.uniform4fv(gl.getUniformLocation(program, "uAmbientProduct"), ambientProduct);
        gl.uniform4fv(gl.getUniformLocation(program, "uDiffuseProduct"), diffuseProduct );
        gl.uniform4fv(gl.getUniformLocation(program, "uSpecularProduct"), specularProduct );
        gl.uniform1f(gl.getUniformLocation(program,"uShininess"), materialShininess);

        // START ANIMATION
        document.getElementById("Start").onclick = function(event) {
            // Clear to avoid multiple executions of the animation
            clearInterval(Animation);
            // startAnimation runs every 150ms
            Animation = setInterval(function() {startAnimation();}, 150);
        };

        // RESET ANIMATION
        document.getElementById("Reset").onclick = function(event) {
            resetAnimation();
        };

        // CONFIGURE/ACTIVE TEXTURES
        var image1 = document.getElementById("texImage1");
        var image2 = document.getElementById("texImage2");
        var image3 = document.getElementById("texImage3");

        // Try configuring the texture as if the images are correctly loaded...
        try{
            configureTexture(image1, image2, image3, normals);
            gl.uniform1f( gl.getUniformLocation(program, "uOkayFlag"), okay );
        }
        // ..If they are not correctly loaded, show error message and render the canvas in basic mode
        catch(err){
            var error_message = document.getElementById("error");
            error_message.innerHTML = "<b>Security Error! Textures are loaded from local data: you must activate a local server to use them.<br>You can activate a local server with the command 'python -m http.server' in terminal.<br>However, I am still going to show you the canvas using basic color textures:</b>";
            okay = false;
            gl.uniform1f( gl.getUniformLocation(program, "uOkayFlag"), okay );

            // jpg images are not available, but I can still generate the bump texture
            bumpTexture = gl.createTexture();
            gl.bindTexture(gl.TEXTURE_2D, bumpTexture);
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, texSize, texSize, 0, gl.RGB, gl.UNSIGNED_BYTE, normals);
            gl.generateMipmap(gl.TEXTURE_2D);
            gl.uniform1i(gl.getUniformLocation(program, "uBumpTextureMap"), 3);
        }

        gl.activeTexture(gl.TEXTURE0);
        gl.bindTexture(gl.TEXTURE_2D, grassTexture);
        gl.activeTexture(gl.TEXTURE1);
        gl.bindTexture(gl.TEXTURE_2D, woolTexture);
        gl.activeTexture(gl.TEXTURE2);
        gl.bindTexture(gl.TEXTURE_2D, fenceTexture);
        gl.activeTexture(gl.TEXTURE3);
        gl.bindTexture(gl.TEXTURE_2D, bumpTexture);

        // Init nodes
        for(i=0; i<numNodes; i++) initNodes(i);

        render();
    }

    // RENDER ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    var render = function() {
            gl.clear( gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
            projectionMatrix = ortho(left, right, bottom, top, near, far);
            var eye = vec3(radius*Math.sin(phi), radius*Math.sin(thetaCam), radius*Math.cos(phi));

            modelViewMatrix = lookAt(eye, at, up);
            modelViewMatrixLoc = gl.getUniformLocation(program, "uModelViewMatrix");

            nMatrix = normalMatrix(modelViewMatrix, true);
            gl.uniformMatrix4fv(modelViewMatrixLoc, false, flatten(modelViewMatrix));
            gl.uniformMatrix4fv( gl.getUniformLocation( program, "uProjectionMatrix"), false, flatten(projectionMatrix));
            gl.uniformMatrix3fv(gl.getUniformLocation(program, "uNormalMatrix"), false, flatten(nMatrix));

            traverse(bodyId);   // Traverse all the sheep's body parts
            traverse(grassId);  // Traverse the grass and the fence (which is grass's child)
            
            requestAnimationFrame(render);
    }
}

// Here the whole execution starts
MyHW2();