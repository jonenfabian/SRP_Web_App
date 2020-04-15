# forms.py
from django import forms
from .models import *

class FileForm(forms.ModelForm):
    class Meta:
        model = SrpUpload #the class!
        fields = ['FI_File_Upload','BOM_File_Upload'] #add more files here
        #fields = ['name']

class DBForm(forms.ModelForm):
    class Meta:
        model = DbUpload #the class!
        fields = ['abs_coordinates_upload','pick_skills_upload','operation_skills_upload'] #add more files here
        #fields = ['name']
