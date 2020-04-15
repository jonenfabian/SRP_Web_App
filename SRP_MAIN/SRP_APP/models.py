from django.db import models

# Create your models here.
class SrpUpload(models.Model):
    #name = models.CharField(max_length=50)
    FI_File_Upload = models.FileField(upload_to='fitting_instructions/')
    BOM_File_Upload = models.FileField(upload_to='bom_files/')
    #Why not have more models like BOM_File?

    #image_from_cam= models.ImageField(upload_to='images/')
    # def __str__(self):
    #     return self.name
#Database Uploads
class DbUpload(models.Model):
    #name = models.CharField(max_length=50)
    abs_coordinates_upload = models.FileField(upload_to='SRP_Database/')
    pick_skills_upload = models.FileField(upload_to='SRP_Database/')
    operation_skills_upload = models.FileField(upload_to='SRP_Database/')
    #Why not have more models like BOM_File?

    #image_from_cam= models.ImageField(upload_to='images/')
    # def __str__(self):
    #     return self.name

class SrpClusterDb(models.Model):
    generated_time=models.TimeField(auto_now=True)
    original=models.CharField(max_length=500)
    operation=models.CharField(max_length=500)
    target=models.CharField(max_length=500)
    relation=models.CharField(max_length=500)
    absolute_coordinates_root_relation=models.TextField()
    absolute_coordinates_root_target=models.TextField()
    custom_pick_skill=models.TextField()
    safety_highway_points=models.TextField()
    target_coordinates_abs=models.TextField()
    safe_move_to_target=models.TextField()
    operation_skill=models.TextField()
    pathplanning_video=models.FileField(upload_to='videos/')

    def get_absolute_url(self):
        return reverse("srpclusterdb_detail",kwargs={'pk':self.pk})
    def __str__(self):
        return self.operation+" "+self.relation+" to "+self.target
